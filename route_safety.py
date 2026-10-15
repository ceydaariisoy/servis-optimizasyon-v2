"""Strict pedestrian/arterial-road stop planning extension for the original project.

No straight-line approximation is ever substituted for a failed pedestrian query.
Road classification is a *screening* test, not a vehicle safety certification.
An approved in-person curbside inspection is required before operating buses.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections import defaultdict
from functools import lru_cache
from urllib.parse import urlencode
from urllib.request import Request, urlopen
import json
import math
import os
import time

def _setting(key: str, default: str) -> str:
    value = os.getenv(key)
    if value:
        return str(value)
    try:
        import streamlit as st
        return str(st.secrets.get(key, default))
    except Exception:
        return default


MAX_WALK_METERS = 1000.0
MAJOR_ROADS = frozenset({"primary", "secondary", "tertiary"})
AUTO_STOP_SOURCE = "Ana yol adayı (saha kontrolü gerekli)"
APPROVED_STOP_SOURCE = "Onaylı durak"
USER_AGENT = "servis-optimizasyon-web-route-safety/2.0 (pedestrian distance checks)"
OVERPASS_ENDPOINT = _setting("OVERPASS_URL", "https://overpass-api.de/api/interpreter")
# A dedicated foot-profile engine is recommended for real datasets. FOSSGIS
# allows <=1 request/second and disallows heavy use: it must be explicitly opted in.
FOOT_ENGINE = _setting("WALKING_OSRM_URL", "").rstrip("/")
MAX_FOOT_REQUESTS = int(_setting("WALKING_MAX_REQUESTS", "48"))
REQUEST_TIMEOUT_S = int(_setting("ROUTE_API_TIMEOUT_S", "30"))


class ValidationError(ValueError):
    """A hard safety rule cannot be satisfied or verified."""


def _request_json(url: str, data: bytes | None = None) -> dict:
    request = Request(url, data=data, headers={"User-Agent": USER_AGENT, "Accept": "application/json"})
    if data is not None:
        request.add_header("Content-Type", "application/x-www-form-urlencoded; charset=utf-8")
    try:
        with urlopen(request, timeout=REQUEST_TIMEOUT_S) as response:
            return json.loads(response.read().decode("utf-8"))
    except Exception as exc:
        raise ValidationError(f"Harita/yaya yolu servisi kullanılamıyor: {exc}") from exc


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    a_lat, a_lon = map(math.radians, a)
    b_lat, b_lon = map(math.radians, b)
    h = math.sin((b_lat-a_lat)/2)**2 + math.cos(a_lat)*math.cos(b_lat)*math.sin((b_lon-a_lon)/2)**2
    return 2 * 6371008.8 * math.asin(min(1., math.sqrt(h)))


def _project_to_segment(point, a, b):
    # Local tangent-plane projection, adequate for segments < several km.
    lat_radians = math.radians(point[0]); sy = 111320.; sx = sy * math.cos(lat_radians)
    ax, ay = (a[1]-point[1])*sx, (a[0]-point[0])*sy
    bx, by = (b[1]-point[1])*sx, (b[0]-point[0])*sy
    dx, dy = bx-ax, by-ay
    t = max(0., min(1., -(ax*dx+ay*dy)/max(1.e-9, dx*dx+dy*dy)))
    result = (float(a[0]+t*(b[0]-a[0])), float(a[1]+t*(b[1]-a[1])))
    return result, haversine_m(point, result)


def _safe_road_tags(tags: dict) -> bool:
    if tags.get("highway") not in MAJOR_ROADS:
        return False
    if any(tags.get(k) in {"no", "private", "agricultural", "forestry"} for k in ("access", "motor_vehicle", "motorcar", "bus", "psv")):
        return False
    if tags.get("tunnel") not in (None, "no"):
        return False
    # Avoid bridge decks and roundabouts as passenger waiting locations.
    if tags.get("bridge") not in (None, "no") or tags.get("junction") == "roundabout":
        return False
    if tags.get("sidewalk") == "no":
        return False
    for name, min_width in (("width", 3.1), ("maxwidth", 3.1), ("maxheight", 3.6), ("maxweight", 15.0)):
        try:
            numeric = float(str(tags.get(name, "")).split()[0].replace(",", "."))
            if numeric < min_width:
                return False
        except (ValueError, IndexError):
            pass
    return True


@dataclass(frozen=True)
class RoadSegment:
    a: tuple[float, float]
    b: tuple[float, float]
    name: str
    highway: str


def parse_overpass_roads(payload: dict) -> list[RoadSegment]:
    segments = []
    for way in payload.get("elements", []):
        tags = way.get("tags", {})
        if way.get("type") != "way" or not _safe_road_tags(tags):
            continue
        coordinates = [(float(n["lat"]), float(n["lon"])) for n in way.get("geometry", []) if "lat" in n and "lon" in n]
        for a, b in zip(coordinates, coordinates[1:]):
            if 1 <= haversine_m(a, b) <= 12000:
                segments.append(RoadSegment(a, b, tags.get("name", "Ana yol"), tags.get("highway", "")))
    return segments


def fetch_major_roads(employee_points: list[tuple[float, float]], extra_points=()) -> list[RoadSegment]:
    points = [*employee_points, *extra_points]
    if not points:
        return []
    # 1.3 km margin on each side, bound oversized requests for safety.
    lat_margin = 0.013; lon_margin = 0.013 / max(.3, math.cos(math.radians(sum(p[0] for p in points) / len(points))))
    south, north = min(p[0] for p in points)-lat_margin, max(p[0] for p in points)+lat_margin
    west, east = min(p[1] for p in points)-lon_margin, max(p[1] for p in points)+lon_margin
    if (north-south) > 0.65 or (east-west) > 0.65:
        raise ValidationError("Çalışanlar tek bir kent bölgesi dışında dağılıyor. Bölge bazında planlama yapın.")
    query = f'[out:json][timeout:45];way["highway"~"^(primary|secondary|tertiary)$"]({south:.6f},{west:.6f},{north:.6f},{east:.6f});out geom;'
    payload = _request_json(OVERPASS_ENDPOINT, data=urlencode({"data":query}).encode())
    if payload.get("remark") and "timed out" in str(payload["remark"]).lower():
        raise ValidationError("Ana yol verisi sorgusu zaman aşımına uğradı.")
    segments = parse_overpass_roads(payload)
    if not segments:
        raise ValidationError("OSM'den uygun ana cadde/ana yol bulunamadı. Harita verisini kontrol edin.")
    return segments


def qualifies_on_main_road(point, segments: list[RoadSegment], max_distance_m=40.) -> bool:
    return any(_project_to_segment(point, s.a, s.b)[1] <= max_distance_m for s in segments)


def propose_major_road_stops(
    employees,
    segments,
    existing,
    limit_per_employee=6,
    max_total_candidates=200,
):
    """Generate a *bounded, coverage-aware* shortlist of road-center stops.

    The former implementation collected up to six candidates per employee and
    hard-failed once the union exceeded 350. With ~80 staff that is common.

    Here we protect every approved stop, first cover employees using a greedy
    shared-stop heuristic, and then spend the remaining candidate budget on
    nearby alternatives. The pedestrian NETWORK later decides what is truly
    reachable: no crow-fly distance is used to approve an assignment.

    Suggested coordinates represent road centrelines, not approved bus stops.
    A safe kerbside location must be checked before field use.
    """
    existing = dict(existing)
    max_total_candidates = max(1, int(max_total_candidates))
    if len(existing) > max_total_candidates:
        raise ValidationError(
            f"Onaylı durak sayısı ({len(existing)}) aday sınırından "
            f"({max_total_candidates}) fazla; önce sınırı artırın veya "
            "kapsamı daraltın. Onaylı duraklar otomatik silinmedi."
        )
    if not employees:
        return list(existing.values())

    # Sample each permitted OSM main-road segment approximately every 220 m.
    samples = {}
    for segment in segments:
        length = haversine_m(segment.a, segment.b)
        steps = max(1, math.ceil(length / 220))
        for step in range(steps + 1):
            point = (
                segment.a[0] + (segment.b[0] - segment.a[0]) * step / steps,
                segment.a[1] + (segment.b[1] - segment.a[1]) * step / steps,
            )
            key = (round(point[0], 5), round(point[1], 5))
            if key not in existing:
                samples.setdefault(key, (point, segment.name))

    # Only collect locally relevant sample keys; avoid unlimited candidates
    # for urban corridors where many parallel main roads are nearby.
    per_employee = []
    distance_lookup = {}
    pool = set()
    for employee_index, employee in enumerate(employees):
        shortlist = []
        for key, (point, _road_name) in samples.items():
            distance_m = haversine_m(employee, point)
            if 110 <= distance_m <= MAX_WALK_METERS:
                shortlist.append((distance_m, key))
        shortlist.sort(key=lambda item: (item[0], item[1]))
        shortlist = shortlist[:max(1, int(limit_per_employee))]
        per_employee.append(shortlist)
        for distance_m, key in shortlist:
            pool.add(key)
            distance_lookup[(employee_index, key)] = distance_m

    selected = dict(existing)
    covered = set()
    for employee_index, employee in enumerate(employees):
        if any(haversine_m(employee, (record[0], record[1])) <= MAX_WALK_METERS
               for record in selected.values()):
            covered.add(employee_index)

    # A stop can serve several colleagues. Greedy coverage avoids producing
    # a new unique stop for each employee merely to bypass an arbitrary cap.
    reverse_coverage = defaultdict(set)
    for employee_index, shortlist in enumerate(per_employee):
        for _, key in shortlist:
            reverse_coverage[key].add(employee_index)
    remaining = set(range(len(employees))) - covered
    available = set(pool)
    while remaining and available and len(selected) < max_total_candidates:
        best = None
        best_key = None
        for key in available:
            newly_covered = reverse_coverage[key] & remaining
            if not newly_covered:
                continue
            walks = [distance_lookup[(i, key)] for i in newly_covered]
            priority = (len(newly_covered), -sum(walks) / len(walks), -min(walks))
            if best is None or priority > best or (priority == best and key < best_key):
                best = priority
                best_key = key
        if best_key is None:
            break
        point, name = samples[best_key]
        selected[best_key] = (
            point[0], point[1], f"{name} — otomatik ana yol adayı", "", AUTO_STOP_SOURCE
        )
        remaining -= reverse_coverage[best_key]
        available.remove(best_key)

    # Do not hide uncovered staff: when geometry cannot even offer a nearby
    # road-center candidate we must ask for a different approved safe stop.
    # The final decision still requires a successful foot-road distance.
    if remaining:
        indices = ', '.join(str(i + 1) for i in sorted(remaining)[:20])
        if len(selected) >= max_total_candidates and any(per_employee[i] for i in remaining):
            raise ValidationError(
                f"Aday durak havuzu {max_total_candidates} sınırına ulaştı ve "
                f"{len(remaining)} çalışan için kapsama tamamlanamadı "
                f"(satırlar: {indices}). Aday limitini kontrollü artırın "
                "veya ulaşım bölgesini ayrı değerlendirin; 1000 m sınırı korunur."
            )
        raise ValidationError(
            f"{len(remaining)} çalışan için 1000 m kuş uçuşu sınırında dahi "
            f"uygun ana yol adayı bulunamadı (çalışan satırları: {indices}). "
            "Güvenli yeni durak veya farklı yol verisi gerekiyor."
        )

    # Add independent near-by choices to protect against bridges, barriers,
    # roundabout detours and disconnected footpaths. We DO NOT treat these
    # aerial approximations as proven pedestrian distances.
    def choices_count(employee_index):
        employee = employees[employee_index]
        return sum(
            haversine_m(employee, (row[0], row[1])) <= MAX_WALK_METERS
            for row in selected.values()
        )

    # Round-robin favours the least-served employee first. Up to three
    # distinct nearby choices are useful for post-network verification.
    for desired_choices in (2, 3):
        if len(selected) >= max_total_candidates:
            break
        order = sorted(range(len(employees)), key=lambda i: (choices_count(i), i))
        for employee_index in order:
            if len(selected) >= max_total_candidates:
                break
            if choices_count(employee_index) >= desired_choices:
                continue
            for _distance, key in per_employee[employee_index]:
                if key in selected:
                    continue
                # Prefer a geometrically distinct alternative; otherwise the
                # foot router could return the same inaccessible road branch.
                if any(haversine_m(samples[key][0], (row[0], row[1])) < 90
                       for row in selected.values()
                       if haversine_m(employees[employee_index], (row[0], row[1])) <= MAX_WALK_METERS):
                    continue
                point, name = samples[key]
                selected[key] = (
                    point[0], point[1], f"{name} — otomatik ana yol adayı", "", AUTO_STOP_SOURCE
                )
                break

    return list(selected.values())


def candidates_on_arterials(employees, approved_candidates, allow_automatic=True, road_segments=None):
    """Return all road-screened candidates as (lat,lon,label,group,source) records.

    Uploaded stops must already have a positive Saha_Onayi in the Excel parser.
    """
    approved_candidates = approved_candidates or []
    points = [(float(p[0]), float(p[1])) for p in approved_candidates if len(p) >= 2]
    segments = road_segments if road_segments is not None else fetch_major_roads(employees, points)
    existing = {}
    rejected = []
    for row in approved_candidates:
        if len(row) < 3:
            continue
        lat, lon, label = float(row[0]), float(row[1]), str(row[2])
        group = str(row[3]) if len(row) > 3 else ""
        if not qualifies_on_main_road((lat, lon), segments):
            rejected.append(label)
            continue
        existing[(round(lat,5), round(lon,5))] = (lat,lon,label,group,APPROVED_STOP_SOURCE)
    if approved_candidates and not existing and not allow_automatic:
        raise ValidationError("Onaylı duraklar arasında ana cadde üzerinde olduğu doğrulanan durak yok.")
    if allow_automatic:
        records = propose_major_road_stops(employees, segments, existing)
    else:
        records = list(existing.values())
    if not records:
        raise ValidationError("Kullanılabilir ana yol durağı yok; onaylı durak ekleyin veya otomatik adaylara izin verin.")
    return records, rejected


class WalkingMatrix:
    """Pedestrian road NETWORK distances, metres. Never falls back to haversine."""
    def __init__(self, endpoint=None, fetch_json=None, max_requests=None):
        self.endpoint = (endpoint if endpoint is not None else FOOT_ENGINE).rstrip("/")
        self.fetch_json = fetch_json or _request_json
        self.max_requests = max_requests if max_requests is not None else MAX_FOOT_REQUESTS
        self.requests = 0
        self.last_request_time = 0.0
        self.cached_pairs = {}
        if not self.endpoint:
            raise ValidationError(
                "Yaya yol ağı yapılandırılmamış: Streamlit Settings → Secrets ekranına "
                "WALKING_OSRM_URL = \"https://routing.openstreetmap.de/routed-foot\" yazın "
                "(pilot test için) veya kuruma ait foot profilli OSRM sunucusunu belirtin. "
                "Kuş uçuşu tahmini kullanılmaz."
            )
        if "router.project-osrm.org" in self.endpoint.lower():
            raise ValidationError("Yaya yönlendirmesine araç OSRM adresi girildi. FOOT/yaya profilli OSRM gereklidir.")

    def distances(self, stops, employees):
        result = [[math.inf] * len(employees) for _ in stops]
        for i, p in enumerate(stops):
            for j, q in enumerate(employees):
                key = (tuple(p), tuple(q))
                if haversine_m(p, q) > MAX_WALK_METERS + 1.0:
                    continue
                if key in self.cached_pairs:
                    result[i][j] = self.cached_pairs[key]
        # Block batched queries, at most 16 stops x 25 employees: moderate URLs.
        for a in range(0, len(stops), 16):
            stop_ids = list(range(a, min(a+16, len(stops))))
            nearby_ids = [j for j, q in enumerate(employees)
                          if any(haversine_m(stops[i], q) <= MAX_WALK_METERS+1 for i in stop_ids)]
            for b in range(0, len(nearby_ids), 25):
                employee_ids = nearby_ids[b:b+25]
                missing = [(i,j) for i in stop_ids for j in employee_ids
                           if haversine_m(stops[i], employees[j]) <= MAX_WALK_METERS+1 and
                           (tuple(stops[i]),tuple(employees[j])) not in self.cached_pairs]
                if not missing:
                    continue
                if self.requests >= self.max_requests:
                    raise ValidationError(f"Yaya matrisi için {self.max_requests} istek sınırı doldu. Yerel foot OSRM kullanın veya bölgeyi bölün.")
                coords = [stops[i] for i in stop_ids] + [employees[j] for j in employee_ids]
                positions = ';'.join(f"{lon:.7f},{lat:.7f}" for lat,lon in coords)
                src = ';'.join(str(k) for k in range(len(stop_ids)))
                dst = ';'.join(str(len(stop_ids)+k) for k in range(len(employee_ids)))
                url = f"{self.endpoint}/table/v1/driving/{positions}?annotations=distance&sources={src}&destinations={dst}"
                # Public routing.openstreetmap.de rate policy: 1 query / sec max.
                if "routing.openstreetmap.de" in self.endpoint:
                    wait = 1.05 - (time.monotonic() - self.last_request_time)
                    if wait > 0: time.sleep(wait)
                payload = self.fetch_json(url)
                self.last_request_time = time.monotonic()
                self.requests += 1
                distances = payload.get("distances")
                if payload.get("code") != "Ok" or not isinstance(distances, list) or len(distances) != len(stop_ids):
                    raise ValidationError("Foot OSRM yol ağı matrisi geçerli yanıt döndürmedi.")
                for row, i in enumerate(stop_ids):
                    if not isinstance(distances[row], list) or len(distances[row]) != len(employee_ids):
                        raise ValidationError("Yaya uzaklık matrisi beklenen boyutta değil.")
                    for column, j in enumerate(employee_ids):
                        raw = distances[row][column]
                        value = float(raw) if raw is not None and math.isfinite(float(raw)) and float(raw) >= 0 else math.inf
                        self.cached_pairs[(tuple(stops[i]),tuple(employees[j]))] = value
                        result[i][j] = value if haversine_m(stops[i], employees[j]) <= MAX_WALK_METERS+1 else math.inf
        return result


def optimize_major_road_stops(employee_coords, approved_candidates, *, capacity: int, allow_automatic=True,
                              target_average_walk_m=400, walking=None, candidate_records=None):
    """Optimizes coverage and pedestrian experience, then passes stops to existing VRP.

    Returns (CommonStop list, generated CandidateStop list, proven) with lazy core import.
    """
    if capacity < 1:
        raise ValidationError("Servis kapasitesi en az bir kişi olmalıdır.")
    if candidate_records is None:
        records, rejected = candidates_on_arterials(employee_coords, approved_candidates, allow_automatic)
    else:
        records, rejected = candidate_records, []
    walker = walking or WalkingMatrix()
    points = [(float(r[0]), float(r[1])) for r in records]
    distances = walker.distances(points, employee_coords)
    employee_choices = []
    missing = []
    for e in range(len(employee_coords)):
        choices = sorted((i for i in range(len(records)) if distances[i][e] <= MAX_WALK_METERS + 1e-9),
                         key=lambda i: (distances[i][e], 0 if records[i][4] == APPROVED_STOP_SOURCE else 1))
        if not choices:
            missing.append(e+1)
        employee_choices.append(choices[:14])
    if missing:
        raise ValidationError(
            f"{len(missing)} çalışan 1000 m yaya yolu içinde ana yol durağına ulaşamıyor. "
            f"Çalışan satır numaraları: {', '.join(map(str,missing[:30]))}. "
            "Güvenli/onaylı yeni durak gerektiğinden sınır gevşetilmedi."
        )
    from ortools.sat.python import cp_model
    from core import CandidateStop, CommonStop
    model = cp_model.CpModel()
    relevant = sorted(set(i for choices in employee_choices for i in choices))
    used = {i:model.new_bool_var(f"stop_{i}") for i in relevant}
    assignments = {}
    for e, choices in enumerate(employee_choices):
        options = []
        for i in choices:
            variable = model.new_bool_var(f"employee_{e}_stop_{i}")
            assignments[(e,i)] = variable
            model.add(variable <= used[i]); options.append(variable)
        model.add(sum(options) == 1)
    # Stop count and total walking cost are jointly optimized. Walks above
    # the operator's average-comfort target and 650m cost progressively more.
    costs = [used[i] * (7500 + (1200 if records[i][4] != APPROVED_STOP_SOURCE else 0)) for i in relevant]
    for (e,i), variable in assignments.items():
        meters = int(math.ceil(distances[i][e]))
        comfort_target = max(100, min(650, int(target_average_walk_m)))
        costs.append(variable * (meters + max(0, meters-comfort_target)*2 + max(0,meters-650)*6))
    model.minimize(sum(costs))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 35
    solver.parameters.num_search_workers = 4
    solver.parameters.random_seed = 42
    status = solver.solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise ValidationError("Gerçek yaya mesafesi kısıtını sağlayan durak eşleştirmesi bulunamadı.")
    by_stop = defaultdict(list)
    for (e,i), variable in assignments.items():
        if solver.value(variable):
            by_stop[i].append((e, distances[i][e]))
    stops = []
    for i in sorted(by_stop):
        r = records[i]
        people = sorted(by_stop[i], key=lambda p:p[1])
        # More than one bus may stop at the SAME physical stop, but each
        # passenger must be assigned to just one vehicle.
        for offset in range(0,len(people),capacity):
            chunk = people[offset:offset+capacity]
            stops.append(CommonStop(anchor_index=i, member_indices=[p[0] for p in chunk],
                         walking_distances_m=[p[1] for p in chunk], latitude=r[0], longitude=r[1],
                         label=r[2], source=r[4], route_group=r[3]))
    candidates = [CandidateStop(latitude=r[0], longitude=r[1], label=r[2],
                                 source=r[4], route_group=r[3]) for r in records]
    return stops, candidates, status == cp_model.OPTIMAL, rejected


def certify_assigned_routes(routes, employee_coords, *, walking=None, max_walk_m=MAX_WALK_METERS):
    """Hard post-solve safeguard; also overwrites display/export walks with actual foot metres."""
    walker = walking or WalkingMatrix()
    pairs = [(stop,idx) for route in routes for stop in route for idx in stop.member_indices]
    if not pairs and employee_coords:
        raise ValidationError("Hiçbir çalışan servise atanamadı.")
    indices = [idx for _,idx in pairs]
    if sorted(indices) != list(range(len(employee_coords))):
        raise ValidationError("Eksik veya mükerrer çalışan ataması tespit edildi.")
    unique_stops = {}
    for stop,_ in pairs:
        key = (float(stop.latitude),float(stop.longitude))
        unique_stops.setdefault(key, len(unique_stops))
    matrix = walker.distances(list(unique_stops), employee_coords)
    errors = []
    for route in routes:
        for stop in route:
            i = unique_stops[(float(stop.latitude),float(stop.longitude))]
            walks = [matrix[i][j] for j in stop.member_indices]
            for employee, distance in zip(stop.member_indices, walks):
                if not math.isfinite(distance) or distance > max_walk_m + 1.e-9:
                    errors.append(f"{employee+1}. kişi: {distance:.0f}m" if math.isfinite(distance) else f"{employee+1}. kişi: yaya bağlantısı yok")
            stop.walking_distances_m = walks
    if errors:
        raise ValidationError("Rota sonrası 1000 m gerçek yaya yolu kontrolü başarısız: " + "; ".join(errors[:25]))
    return {"max_walk_m": max((w for route in routes for s in route for w in s.walking_distances_m), default=0),
            "avg_walk_m": sum((w for route in routes for s in route for w in s.walking_distances_m))/len(employee_coords) if employee_coords else 0}
