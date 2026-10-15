from __future__ import annotations

from io import BytesIO
import hashlib
import math
import re

import pandas as pd
import pydeck as pdk
import streamlit as st

from core import (
    CommonStop,
    assign_common_stops_to_routes,
    fetch_osrm_geometry,
    generate_candidate_stops,
    get_travel_matrices,
    optimize_candidate_stops,
    reverse_routes_for_return,
    update_routes_incrementally,
)
from route_safety import (
    MAX_WALK_METERS,
    WalkingMatrix,
    optimize_major_road_stops,
    certify_assigned_routes,
    ValidationError,
)


APP_VERSION = "2026.10-major-road-pedestrian-v1.2-502-recovery"
FIXED_TARGET_AVERAGE_WALK_M = 400
FIXED_WAIT_SECONDS_PER_STOP = 15
MORNING_FACTORY_ARRIVAL_SECONDS = 7 * 3600 + 55 * 60
EVENING_FACTORY_DEPARTURE_SECONDS = 17 * 3600 + 40 * 60


st.set_page_config(
    page_title="Servis Rota Optimizasyonu",
    page_icon="🚌",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown(
    """
    <style>
    :root {
        --navy: #17324D;
        --blue: #285A84;
        --blue-soft: #EAF1F7;
        --ink: #1F2937;
        --muted: #667085;
        --line: #DCE3EA;
        --surface: #FFFFFF;
        --background: #F6F8FA;
        --success: #147D64;
    }

    .stApp {background: var(--background);}
    .block-container {
        max-width: 1480px;
        padding-top: 1.4rem;
        padding-bottom: 4rem;
    }
    #MainMenu, footer {visibility: hidden;}
    [data-testid="stToolbar"] {display: none;}
    [data-testid="stSidebar"] {
        background: #F1F4F7;
        border-right: 1px solid var(--line);
    }
    [data-testid="stSidebarContent"] {padding-top: 1.1rem;}
    [data-testid="stSidebar"] hr {margin: 0.9rem 0;}
    [data-testid="stSidebar"] .stCaption {color: #687587;}

    .app-hero {
        display: flex;
        align-items: center;
        gap: 1.15rem;
        padding: 1.35rem 1.5rem;
        margin-bottom: 1.6rem;
        background: linear-gradient(120deg, #FFFFFF 0%, #F3F7FA 100%);
        border: 1px solid var(--line);
        border-radius: 18px;
        box-shadow: 0 10px 30px rgba(23, 50, 77, 0.06);
    }
    .hero-icon {
        width: 54px;
        height: 54px;
        flex: 0 0 54px;
        display: grid;
        place-items: center;
        background: var(--navy);
        border-radius: 14px;
        color: #FFFFFF;
    }
    .hero-kicker {
        color: var(--blue);
        font-size: 0.72rem;
        font-weight: 750;
        letter-spacing: 0.11em;
        margin-bottom: 0.18rem;
    }
    .app-hero h1 {
        color: var(--ink);
        font-size: clamp(1.75rem, 3vw, 2.45rem);
        line-height: 1.15;
        letter-spacing: -0.025em;
        margin: 0;
    }
    .app-hero p {
        color: var(--muted);
        font-size: 0.98rem;
        line-height: 1.55;
        margin: 0.4rem 0 0;
    }

    .section-heading {
        display: flex;
        align-items: flex-start;
        gap: 0.85rem;
        margin: 1.75rem 0 0.85rem;
    }
    .step-badge {
        min-width: 38px;
        height: 30px;
        display: grid;
        place-items: center;
        color: var(--blue);
        background: var(--blue-soft);
        border-radius: 9px;
        font-size: 0.76rem;
        font-weight: 800;
        letter-spacing: 0.04em;
    }
    .section-heading h2 {
        color: var(--ink);
        font-size: 1.3rem;
        line-height: 1.25;
        margin: 0;
    }
    .section-heading p {
        color: var(--muted);
        font-size: 0.88rem;
        line-height: 1.45;
        margin: 0.18rem 0 0;
    }

    .sidebar-brand {
        padding: 0.35rem 0 0.7rem;
    }
    .sidebar-brand strong {
        display: block;
        color: var(--navy);
        font-size: 1.08rem;
    }
    .sidebar-brand span {
        color: var(--muted);
        font-size: 0.79rem;
    }
    .sidebar-note {
        color: #607083;
        background: #FFFFFF;
        border: 1px solid var(--line);
        border-radius: 10px;
        padding: 0.7rem 0.8rem;
        font-size: 0.78rem;
        line-height: 1.45;
        margin-top: 0.6rem;
    }

    div[data-testid="stMetric"] {
        min-height: 104px;
        background: var(--surface);
        border: 1px solid var(--line);
        border-radius: 13px;
        padding: 0.9rem 1rem;
        box-shadow: 0 4px 14px rgba(23, 50, 77, 0.035);
    }
    div[data-testid="stMetric"] label {
        color: var(--muted);
        font-size: 0.82rem;
        font-weight: 600;
    }
    div[data-testid="stMetricValue"] {
        color: var(--navy);
        font-size: 1.75rem;
    }
    [data-testid="stFileUploader"] {
        background: var(--surface);
        border: 1px solid var(--line);
        border-radius: 13px;
        padding: 0.85rem 1rem 0.35rem;
    }
    [data-testid="stFileUploaderDropzone"] {
        background: #FAFBFC;
        border-color: #B9C7D4;
        border-radius: 10px;
    }
    [data-testid="stExpander"] {
        background: var(--surface);
        border-color: var(--line);
        border-radius: 11px;
    }
    [data-testid="stAlert"] {border-radius: 11px;}
    .stButton > button[kind="primary"],
    .stDownloadButton > button[kind="primary"] {
        min-height: 2.85rem;
        background: var(--navy);
        border-color: var(--navy);
        border-radius: 10px;
        font-weight: 700;
        box-shadow: 0 6px 16px rgba(23, 50, 77, 0.14);
    }
    .stButton > button[kind="primary"]:hover,
    .stDownloadButton > button[kind="primary"]:hover {
        background: #214767;
        border-color: #214767;
    }

    .route-card {
        background: var(--surface);
        border: 1px solid var(--line);
        border-left: 5px solid var(--blue);
        padding: 1rem 1.15rem;
        border-radius: 13px;
        margin: 0.85rem 0 0.55rem;
        box-shadow: 0 5px 16px rgba(23, 50, 77, 0.04);
    }
    .route-card-head {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 1rem;
        margin-bottom: 0.85rem;
    }
    .route-title {
        color: var(--navy);
        font-size: 1rem;
        font-weight: 800;
    }
    .route-occupancy {
        color: var(--blue);
        background: var(--blue-soft);
        border-radius: 999px;
        padding: 0.3rem 0.65rem;
        font-size: 0.76rem;
        font-weight: 750;
        white-space: nowrap;
    }
    .route-stat-grid {
        display: grid;
        grid-template-columns: repeat(6, minmax(80px, 1fr));
        gap: 0.75rem;
    }
    .route-stat strong {
        display: block;
        color: var(--ink);
        font-size: 0.92rem;
    }
    .route-stat span {
        color: var(--muted);
        font-size: 0.72rem;
    }
    .muted {color: var(--muted); font-size: 0.92rem;}

    @media (max-width: 900px) {
        .app-hero {padding: 1.1rem;}
        .hero-icon {width: 46px; height: 46px; flex-basis: 46px;}
        .route-stat-grid {grid-template-columns: repeat(2, 1fr);}
    }
    </style>
    """,
    unsafe_allow_html=True,
)

st.markdown(
    """
    <div class="app-hero">
        <div class="hero-icon" aria-hidden="true">
            <svg width="29" height="29" viewBox="0 0 24 24" fill="none"
                 xmlns="http://www.w3.org/2000/svg">
                <path d="M4 17.5V8.8C4 6.7 5.7 5 7.8 5h8.4C18.3 5 20 6.7 20 8.8v8.7"
                      stroke="currentColor" stroke-width="1.7" stroke-linecap="round"/>
                <path d="M4 14.5h16M7 9h3.2M13.8 9H17"
                      stroke="currentColor" stroke-width="1.7" stroke-linecap="round"/>
                <circle cx="7" cy="18" r="1.7" fill="currentColor"/>
                <circle cx="17" cy="18" r="1.7" fill="currentColor"/>
            </svg>
        </div>
        <div>
            <div class="hero-kicker">OPERASYON PLANLAMA</div>
            <h1>Servis Rota Optimizasyonu</h1>
            <p>Çalışan ve durak verilerini yükleyin; kapasite, süre ve yürüme
               kısıtlarına göre uygulanabilir servis rotalarını oluşturun.</p>
        </div>
    </div>
    """,
    unsafe_allow_html=True,
)



def _local_haversine_km(a: tuple[float, float], b: tuple[float, float]) -> float:
    """İki koordinat arasındaki kuş uçuşu mesafeyi km olarak hesaplar."""
    radius_km = 6371.0088
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    return 2 * radius_km * math.asin(math.sqrt(h))


def consolidate_allocated_routes(
    routes,
    employee_coordinates,
    duration_matrix,
    max_walk_m,
    wait_seconds_per_stop,
    max_route_minutes=0,
    walking_factor=1.20,
    max_extra_walk_m=180.0,
    absolute_walk_cap_m=900.0,
    max_stop_distance_m=1400.0,
    max_passes=20,
):
    """Ana rota dağılımını bozmadan aynı rota içindeki yakın durakları birleştirir."""

    automatic_sources = {
        "Güzergâh üzeri aday durak",
        "Güzergâha yakın yeni durak",
        "Otomatik ortak nokta",
    }

    def clone_stop(stop):
        cloned = CommonStop(
            anchor_index=stop.anchor_index,
            member_indices=list(stop.member_indices),
            walking_distances_m=list(stop.walking_distances_m),
            latitude=float(stop.latitude),
            longitude=float(stop.longitude),
            label=stop.label,
            source=stop.source,
            matrix_index=stop.matrix_index,
        )
        if hasattr(cloned, "route_group") and hasattr(stop, "route_group"):
            cloned.route_group = getattr(stop, "route_group", "")
        return cloned

    working_routes = [
        [clone_stop(stop) for stop in route]
        for route in routes
    ]

    def idx(stop):
        return int(
            stop.matrix_index
            if stop.matrix_index is not None
            else stop.anchor_index + 1
        )

    def directional_seconds(route):
        if not route:
            return 0.0, 0.0

        morning_indices = [idx(stop) for stop in route]
        morning_path = [*morning_indices, 0]
        morning_drive = sum(
            float(duration_matrix[a][b])
            for a, b in zip(morning_path, morning_path[1:])
        )

        evening_indices = list(reversed(morning_indices))
        evening_path = [0, *evening_indices]
        evening_drive = sum(
            float(duration_matrix[a][b])
            for a, b in zip(evening_path, evening_path[1:])
        )

        dwell = len(route) * float(wait_seconds_per_stop)
        return morning_drive + dwell, evening_drive + dwell

    def current_walks(route):
        values = {}
        for stop in route:
            for employee_index, walk in zip(
                stop.member_indices,
                stop.walking_distances_m,
            ):
                values[employee_index] = float(walk)
        return values

    total_merged = 0
    moved_employees = 0
    route_merge_counts = []

    for route_index, original_route in enumerate(working_routes):
        route = original_route
        merges = 0

        for _ in range(max(0, int(max_passes))):
            if len(route) <= 1:
                break

            baseline_morning, baseline_evening = directional_seconds(route)
            old_walk_map = current_walks(route)
            best = None

            for source_pos, source in enumerate(route):
                for target_pos, target in enumerate(route):
                    if source_pos == target_pos:
                        continue

                    stop_distance_m = (
                        _local_haversine_km(
                            (float(source.latitude), float(source.longitude)),
                            (float(target.latitude), float(target.longitude)),
                        )
                        * 1000.0
                    )
                    if stop_distance_m > max_stop_distance_m + 1e-9:
                        continue

                    new_walks = []
                    total_extra = 0.0
                    largest_extra = 0.0
                    feasible = True

                    for employee_index in source.member_indices:
                        emp = employee_coordinates[employee_index]
                        new_walk = (
                            _local_haversine_km(
                                (float(target.latitude), float(target.longitude)),
                                (float(emp[0]), float(emp[1])),
                            )
                            * 1000.0
                            * walking_factor
                        )
                        old_walk = float(
                            old_walk_map.get(employee_index, new_walk)
                        )

                        if old_walk > absolute_walk_cap_m:
                            allowed_walk = old_walk
                        else:
                            allowed_walk = min(
                                float(max_walk_m),
                                float(absolute_walk_cap_m),
                                old_walk + float(max_extra_walk_m),
                            )

                        if (
                            new_walk > float(max_walk_m) + 1e-9
                            or new_walk > allowed_walk + 1e-9
                        ):
                            feasible = False
                            break

                        extra = max(0.0, new_walk - old_walk)
                        total_extra += extra
                        largest_extra = max(largest_extra, extra)
                        new_walks.append(new_walk)

                    if not feasible:
                        continue

                    trial = [clone_stop(stop) for stop in route]
                    trial_source = trial[source_pos]

                    adjusted_target = (
                        target_pos - 1
                        if source_pos < target_pos
                        else target_pos
                    )
                    del trial[source_pos]
                    trial_target = trial[adjusted_target]

                    trial_target.member_indices.extend(
                        trial_source.member_indices
                    )
                    trial_target.walking_distances_m.extend(
                        new_walks
                    )

                    pairs = sorted(
                        zip(
                            trial_target.member_indices,
                            trial_target.walking_distances_m,
                        ),
                        key=lambda pair: (pair[1], pair[0]),
                    )
                    trial_target.member_indices = [
                        pair[0] for pair in pairs
                    ]
                    trial_target.walking_distances_m = [
                        pair[1] for pair in pairs
                    ]

                    trial_morning, trial_evening = directional_seconds(trial)

                    # Rota süresi iki yönde de artmayacak.
                    if trial_morning > baseline_morning + 1e-9:
                        continue
                    if trial_evening > baseline_evening + 1e-9:
                        continue

                    if max_route_minutes:
                        limit = float(max_route_minutes) * 60.0
                        if (
                            trial_morning > limit + 1e-9
                            or trial_evening > limit + 1e-9
                        ):
                            continue

                    time_saved = (
                        baseline_morning
                        + baseline_evening
                        - trial_morning
                        - trial_evening
                    )

                    priority = (
                        1 if source.source in automatic_sources else 0,
                        1 if source.passenger_count == 1 else 0,
                        -source.passenger_count,
                        time_saved,
                        -total_extra,
                        -largest_extra,
                        target.passenger_count,
                    )

                    if best is None or priority > best[0]:
                        best = (
                            priority,
                            trial,
                            len(source.member_indices),
                        )

            if best is None:
                break

            _, route, moved_now = best
            merges += 1
            total_merged += 1
            moved_employees += moved_now

        working_routes[route_index] = route
        route_merge_counts.append(merges)

    return working_routes, {
        "merged_stop_count": total_merged,
        "moved_employee_count": moved_employees,
        "route_merge_counts": route_merge_counts,
        "post_merge_stop_count": sum(
            len(route) for route in working_routes
        ),
    }



AUTOMATIC_STOP_SOURCES = {
    "Otomatik ortak nokta",
    "Güzergâh üzeri aday durak",
    "Güzergâha yakın yeni durak",
    "Ana yol adayı (saha kontrolü gerekli)",
}


def professional_stop_name(stop: dict, route_no: int, order: int) -> str:
    """Durak adını kullanıcıya daha kurumsal ve anlaşılır biçimde gösterir.

    Optimizasyonda kullanılan gerçek label değiştirilmez. Bu fonksiyon yalnızca
    ekran, harita ve rapor çıktısında sunulan adı düzenler.
    """
    raw = str(stop.get("label", "") or "").strip()
    source = str(stop.get("source", "") or "").strip()

    if source not in AUTOMATIC_STOP_SOURCES:
        return raw

    # Eski koridor adlarını kullanıcıya göstermemek için baştaki
    # "VitrA Karo 1/2/3" ifadesini temizle.
    cleaned = re.sub(
        r"^\s*VitrA\s*Karo\s*\d+\s*",
        "",
        raw,
        flags=re.IGNORECASE,
    ).strip()

    # Yol/sokak adı OSRM'den geldiyse onu koru.
    road_name = re.sub(
        r"\s*(güzergâha yakın yeni durak|güzergâh üzeri aday durak|"
        r"ortak güzergâh adayı(?:\s*çift)?|güzergâh adayı)\s*\d*\s*$",
        "",
        cleaned,
        flags=re.IGNORECASE,
    ).strip(" -–")

    generic_tokens = {
        "",
        "ortak",
        "yeni durak",
        "aday durak",
    }
    has_meaningful_road_name = road_name.casefold() not in generic_tokens

    if source == "Otomatik ortak nokta":
        if has_meaningful_road_name:
            return f"{road_name} – Önerilen Ortak Durak"
        return f"Önerilen Ortak Durak R{route_no}-{order}"

    if source == "Güzergâh üzeri aday durak":
        if has_meaningful_road_name:
            return f"{road_name} – Önerilen Güzergâh Durağı"
        return f"Önerilen Güzergâh Durağı R{route_no}-{order}"

    if source == "Güzergâha yakın yeni durak":
        if has_meaningful_road_name:
            return f"{road_name} – Önerilen Yeni Durak"
        return f"Önerilen Yeni Durak R{route_no}-{order}"

    return raw


def professional_stop_source(source: object) -> str:
    source = str(source or "").strip()
    mapping = {
        "Yüklenen aday durak": "Mevcut / Yüklenen Durak",
        "Onaylı durak": "Mevcut / Onaylı Durak",
        "Mevcut durak": "Mevcut Durak",
        "Otomatik ortak nokta": "Yeni Ortak Durak Önerisi",
        "Güzergâh üzeri aday durak": "Yeni Güzergâh Durağı Önerisi",
        "Güzergâha yakın yeni durak": "Yeni Güzergâh Durağı Önerisi",
        "Ana yol adayı (saha kontrolü gerekli)": "Ana Yol Durağı Önerisi (Saha Onayı Gerekli)",
    }
    return mapping.get(source, source)


def stop_field_check(source: object) -> str:
    return "Gerekli" if str(source or "").strip() in AUTOMATIC_STOP_SOURCES else "-"


def section_header(step: str, title: str, description: str) -> None:
    st.markdown(
        f"""
        <div class="section-heading">
            <div class="step-badge">{step}</div>
            <div>
                <h2>{title}</h2>
                <p>{description}</p>
            </div>
        </div>
        """,
        unsafe_allow_html=True,
    )


FACTORY_ADDRESS = (
    "VitrA Karo Sanayi ve Ticaret A.Ş., 4 Eylül Mah. "
    "Osman Rusçuk Cd. No:13, Bozüyük, Bilecik"
)
# Kampüs merkezi için yaklaşık koordinat. Servis kapısı biliniyorsa arayüzden değiştirilebilir.
FACTORY_LAT = 39.903830
FACTORY_LON = 30.084850


ALIASES = {
    "id": ["calisan_id", "çalışan_id", "personel_sicil", "sicil", "id"],
    "name": ["ad_soyad", "ad soyad", "çalışanın_adı_soyad", "calisan_adi", "çalışan adı"],
    "type": ["tip", "lokasyon_tipi", "nokta_tipi"],
    "address": ["adres", "acik_adres", "açık_adres", "ikamet_adresi"],
    "lat": ["enlem", "latitude", "lat"],
    "lon": ["boylam", "longitude", "lon", "lng"],
    "active": ["aktif_mi", "aktif mi", "aktif"],
    "uses_service": ["servis_kullaniyor_mu", "servis kullanıyor mu", "servis_kullanimi", "servis"],
}


def normalize(value: object) -> str:
    table = str.maketrans("çğıöşüÇĞİÖŞÜ", "cgiosuCGIOSU")
    return str(value).strip().translate(table).casefold().replace(" ", "_")


def detect_column(columns, alias_key):
    normalized = {normalize(column): column for column in columns}
    for alias in ALIASES[alias_key]:
        match = normalized.get(normalize(alias))
        if match is not None:
            return match
    return None


def is_yes(value: object, default: bool = True) -> bool:
    if pd.isna(value) or str(value).strip() == "":
        return default
    return normalize(value) in {"evet", "e", "yes", "true", "1", "aktif", "kullaniyor"}


def is_factory(value: object) -> bool:
    text = normalize(value)
    return any(word in text for word in ("ofis", "fabrika", "depo", "is_yeri"))


def get_mapping(df: pd.DataFrame):
    columns = list(df.columns)
    none_label = "(Yok)"
    options = [none_label, *columns]

    def select(label, key, required=False):
        detected = detect_column(columns, key)
        default = options.index(detected) if detected in options else 0
        value = st.selectbox(label, options, index=default, key=f"map_{key}")
        if required and value == none_label:
            st.warning(f"'{label}' alanını seçmelisiniz.")
        return None if value == none_label else value

    with st.expander("Gelişmiş: Excel sütun eşleştirmesini kontrol et", expanded=False):
        c1, c2, c3 = st.columns(3)
        with c1:
            id_col = select("Çalışan / sicil no", "id")
            name_col = select("Ad soyad", "name")
            type_col = select("Nokta tipi", "type")
        with c2:
            address_col = select("Adres", "address")
            lat_col = select("Enlem", "lat")
            lon_col = select("Boylam", "lon")
        with c3:
            active_col = select("Aktif mi?", "active")
            service_col = select("Servis kullanıyor mu?", "uses_service")
    return {
        "id": id_col,
        "name": name_col,
        "type": type_col,
        "address": address_col,
        "lat": lat_col,
        "lon": lon_col,
        "active": active_col,
        "uses_service": service_col,
    }


def standardize(df: pd.DataFrame, mapping: dict) -> pd.DataFrame:
    df = df.dropna(how="all").copy()
    out = pd.DataFrame(index=df.index)
    id_values = (
        df[mapping["id"]]
        if mapping["id"]
        else pd.Series(range(len(df)), index=df.index)
    )
    out["Calisan_ID"] = id_values.fillna("").astype(str).str.strip()
    out["Ad_Soyad"] = df[mapping["name"]].fillna("").astype(str) if mapping["name"] else ""
    out["Tip"] = df[mapping["type"]].fillna("Çalışan").astype(str) if mapping["type"] else "Çalışan"
    out["Adres"] = df[mapping["address"]].fillna("").astype(str) if mapping["address"] else ""
    out["Enlem"] = pd.to_numeric(df[mapping["lat"]], errors="coerce") if mapping["lat"] else math.nan
    out["Boylam"] = pd.to_numeric(df[mapping["lon"]], errors="coerce") if mapping["lon"] else math.nan
    out["Aktif_mi"] = df[mapping["active"]].apply(is_yes) if mapping["active"] else True
    out["Servis_Kullaniyor_mu"] = (
        df[mapping["uses_service"]].apply(is_yes) if mapping["uses_service"] else True
    )
    return out


def read_approved_candidates(uploaded_file) -> tuple[list[tuple], dict]:
    """Durak Excel'ini okur; rota/grup bilgisini korur ve reddedilenleri dışlar."""
    if uploaded_file is None:
        return [], {"loaded": 0, "approved": 0, "pending": 0, "excluded": 0}
    frame = pd.read_excel(BytesIO(uploaded_file.getvalue())).dropna(how="all")
    lat_col = detect_column(frame.columns, "lat")
    lon_col = detect_column(frame.columns, "lon")
    if lat_col is None or lon_col is None:
        raise ValueError("Durak dosyasında `Enlem` ve `Boylam` sütunları bulunmalıdır.")
    normalized = {normalize(column): column for column in frame.columns}
    name_col = next(
        (
            normalized[key]
            for key in ("durak_adi", "durak_adı", "durak", "stop_name", "name")
            if key in normalized
        ),
        None,
    )
    route_col = next(
        (
            normalized[key]
            for key in ("mevcut_rota", "rota", "route", "hat", "servis_hatti")
            if key in normalized
        ),
        None,
    )
    status_col = normalized.get("saha_onayi") or normalized.get("onay_durumu")
    candidates: list[tuple] = []
    stats = {"loaded": 0, "approved": 0, "pending": 0, "excluded": 0}
    for row_no, (_, row) in enumerate(frame.iterrows(), start=1):
        lat = pd.to_numeric(row[lat_col], errors="coerce")
        lon = pd.to_numeric(row[lon_col], errors="coerce")
        if pd.isna(lat) or pd.isna(lon):
            continue
        status = normalize(row[status_col]) if status_col and not pd.isna(row[status_col]) else ""
        if status in {"uygun_degil", "reddedildi", "red", "hayir", "kullanilmayacak"}:
            stats["excluded"] += 1
            continue
        if status not in {"onaylandi", "onayli", "evet", "uygun"}:
            stats["pending"] += 1
            continue
        label = str(row[name_col]).strip() if name_col and not pd.isna(row[name_col]) else f"Yüklenen durak {row_no}"
        route_group = str(row[route_col]).strip() if route_col and not pd.isna(row[route_col]) else ""
        # Dördüncü alan mevcut rota bilgisidir. core.py bu bilgiyi yalnızca
        # güzergâh segmentlerini doğru kurmak için kullanır; durak adı değişmez.
        candidates.append((float(lat), float(lon), label, route_group))
        stats["loaded"] += 1
        if status in {"onaylandi", "onayli", "evet", "uygun"}:
            stats["approved"] += 1
        else:
            stats["pending"] += 1
    if not candidates and not stats["excluded"] and not stats["pending"]:
        raise ValueError("Durak dosyasında geçerli koordinat bulunamadı.")
    return candidates, stats

def normalize_employee_id(value: object) -> str:
    """Excel'in 123, 123.0 ve metin biçimlerini aynı sicil olarak eşleştirir."""
    if pd.isna(value):
        return ""
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    text = str(value).strip()
    if re.fullmatch(r"-?\d+\.0+", text):
        return text.split(".", 1)[0]
    return text.casefold()


def route_number(value: object) -> int:
    match = re.search(r"\d+", str(value))
    if not match:
        raise ValueError(f"Önceki sonuç dosyasında rota numarası okunamadı: {value}")
    return int(match.group())



def _walking_distance_m(
    first: tuple[float, float],
    second: tuple[float, float],
    walking_factor: float = 1.20,
) -> float:
    """İki koordinat arasındaki yaklaşık yürüme mesafesini metre olarak hesaplar."""
    earth_radius_m = 6_371_008.8
    lat1, lon1 = map(math.radians, first)
    lat2, lon2 = map(math.radians, second)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    straight_m = 2 * earth_radius_m * math.asin(min(1.0, math.sqrt(h)))
    return straight_m * walking_factor


def _stop_key(latitude: float, longitude: float, label: object) -> tuple:
    """Aynı fiziksel yüklenen durağı güvenli biçimde eşleştirmek için anahtar."""
    return (
        round(float(latitude), 7),
        round(float(longitude), 7),
        normalize(label),
    )


def enforce_nearest_loaded_stop_assignments(
    stops: list[CommonStop],
    employee_coordinates: list[tuple[float, float]],
    approved_candidates: list[tuple],
    max_walk_m: float,
    walking_factor: float = 1.20,
) -> list[CommonStop]:
    """
    Çalışan yürüme sınırı içinde yüklenen/mevcut bir durağa ulaşabiliyorsa
    onu erişebildiği EN YAKIN yüklenen durağa atar.

    Böylece sistem sırf durak sayısını azaltmak için çalışanı daha uzaktaki
    mevcut durağa göndermez. Yüklenen durağa yürüyemeyen çalışanların
    güzergâh üzeri / güzergâha yakın otomatik durak eşleşmeleri korunur.
    """
    loaded: list[tuple[float, float, str]] = []
    for raw in approved_candidates or []:
        if len(raw) < 3:
            continue
        try:
            lat = float(raw[0])
            lon = float(raw[1])
        except (TypeError, ValueError):
            continue
        label = str(raw[2]).strip() or "Yüklenen aday durak"
        loaded.append((lat, lon, label))

    if not loaded:
        return stops

    nearest_loaded: dict[int, tuple[float, float, str, float]] = {}
    for employee_index, employee_point in enumerate(employee_coordinates):
        feasible: list[tuple[float, float, str, float]] = []
        for lat, lon, label in loaded:
            walk_m = _walking_distance_m(
                (lat, lon),
                employee_point,
                walking_factor=walking_factor,
            )
            if walk_m <= max_walk_m + 1e-9:
                feasible.append((lat, lon, label, walk_m))
        if feasible:
            nearest_loaded[employee_index] = min(
                feasible,
                key=lambda item: (item[3], normalize(item[2])),
            )

    if not nearest_loaded:
        return stops

    # Mevcut optimizasyonun bu çalışanlar için yaptığı atamaları çıkar.
    cleaned_stops: list[CommonStop] = []
    for stop in stops:
        kept_pairs = [
            (employee_index, walk_m)
            for employee_index, walk_m in zip(
                stop.member_indices,
                stop.walking_distances_m,
            )
            if employee_index not in nearest_loaded
        ]
        if not kept_pairs:
            continue
        stop.member_indices = [pair[0] for pair in kept_pairs]
        stop.walking_distances_m = [pair[1] for pair in kept_pairs]
        cleaned_stops.append(stop)

    # Seçili duraklar içinde zaten bulunan yüklenen durakları eşleştir.
    stop_by_key: dict[tuple, CommonStop] = {}
    for stop in cleaned_stops:
        if stop.latitude is None or stop.longitude is None:
            continue
        if stop.source in {"Yüklenen aday durak", "Onaylı durak", "Mevcut durak"}:
            stop_by_key[
                _stop_key(stop.latitude, stop.longitude, stop.label)
            ] = stop

    # Her çalışanı kendi en yakın yüklenen durağına ekle.
    for employee_index, (lat, lon, label, walk_m) in nearest_loaded.items():
        key = _stop_key(lat, lon, label)
        target_stop = stop_by_key.get(key)
        if target_stop is None:
            target_stop = CommonStop(
                anchor_index=len(cleaned_stops),
                member_indices=[],
                walking_distances_m=[],
                latitude=lat,
                longitude=lon,
                label=label,
                source="Yüklenen aday durak",
            )
            cleaned_stops.append(target_stop)
            stop_by_key[key] = target_stop

        target_stop.member_indices.append(employee_index)
        target_stop.walking_distances_m.append(walk_m)

    # Durağın içindeki çalışanları yürüme mesafesine göre sırala.
    for stop in cleaned_stops:
        pairs = sorted(
            zip(stop.member_indices, stop.walking_distances_m),
            key=lambda pair: (pair[1], pair[0]),
        )
        stop.member_indices = [pair[0] for pair in pairs]
        stop.walking_distances_m = [pair[1] for pair in pairs]

    return cleaned_stops


def is_forbidden_home_stop(label: object, source: object) -> bool:
    """Eski sonuç dosyalarındaki ev/adres bazlı durakları yeni plana taşıma."""
    text = normalize(f"{label} {source}")
    forbidden_terms = (
        "calisan_adresi",
        "ev_adresi",
        "adres_tabanli_yedek_durak",
    )
    return any(term in text for term in forbidden_terms)


def read_previous_routes(uploaded_file, employees: pd.DataFrame) -> list[list[CommonStop]]:
    """Uygulamanın indirdiği sonuç Excel'ini artımlı plan için geri okur."""
    if uploaded_file is None:
        raise ValueError(
            "Mevcut planı koruma modu için daha önce indirdiğiniz "
            "`servis_rota_sonuclari.xlsx` dosyasını yükleyin."
        )

    raw = uploaded_file.getvalue() if hasattr(uploaded_file, "getvalue") else bytes(uploaded_file)

    try:
        stops = pd.read_excel(BytesIO(raw), sheet_name="Ortak_Duraklar").dropna(how="all")
        assignments = pd.read_excel(
            BytesIO(raw),
            sheet_name="Personel_Durak_Eslesmesi",
        ).dropna(how="all")
    except Exception as exc:
        raise ValueError(
            "Önceki plan dosyası, uygulamadan indirilen rota sonuç Excel'i olmalıdır."
        ) from exc

    required_stops = {"Rota", "Durak_Sirasi", "Durak_Adi", "Enlem", "Boylam"}
    required_assignments = {"Rota", "Durak_Sirasi", "Calisan_ID"}

    if (
        not required_stops.issubset(stops.columns)
        or not required_assignments.issubset(assignments.columns)
    ):
        raise ValueError(
            "Önceki rota sonuç dosyasının gerekli sayfa veya sütunları eksik."
        )

    current_keys = [
        normalize_employee_id(value)
        for value in employees["Calisan_ID"]
    ]

    if any(not key for key in current_keys):
        raise ValueError(
            "Mevcut rotayı korumak için her çalışanın sicil/ID bilgisi dolu olmalıdır."
        )

    if len(set(current_keys)) != len(current_keys):
        raise ValueError(
            "Mevcut rotayı korumak için çalışan sicil/ID değerleri benzersiz olmalıdır."
        )

    employee_by_key = {
        key: index
        for index, key in enumerate(current_keys)
    }

    members_by_stop: dict[tuple[int, int], list[int]] = {}

    for _, row in assignments.iterrows():
        key = normalize_employee_id(row["Calisan_ID"])

        if key not in employee_by_key:
            continue

        stop_key = (
            route_number(row["Rota"]),
            int(row["Durak_Sirasi"]),
        )

        members_by_stop.setdefault(
            stop_key,
            [],
        ).append(
            employee_by_key[key]
        )

    route_map: dict[int, list[CommonStop]] = {}

    ordered_stops = stops.sort_values(
        ["Rota", "Durak_Sirasi"],
        kind="stable",
    )

    for _, row in ordered_stops.iterrows():

        lat = pd.to_numeric(
            row["Enlem"],
            errors="coerce",
        )

        lon = pd.to_numeric(
            row["Boylam"],
            errors="coerce",
        )

        if pd.isna(lat) or pd.isna(lon):
            continue

        number = route_number(row["Rota"])
        order = int(row["Durak_Sirasi"])

        label = str(
            row["Durak_Adi"]
        ).strip()

        source = (
            str(row["Durak_Kaynagi"]).strip()
            if (
                "Durak_Kaynagi" in row
                and not pd.isna(row["Durak_Kaynagi"])
            )
            else "Önceki plan"
        )

        # Eski planlarda bulunan çalışan-adresi / adres tabanlı yedek
        # durakları yeni plana taşımıyoruz.
        if is_forbidden_home_stop(
            label,
            source,
        ):
            continue

        members = list(
            dict.fromkeys(
                members_by_stop.get(
                    (number, order),
                    [],
                )
            )
        )

        route_map.setdefault(
            number,
            [],
        ).append(
            CommonStop(
                anchor_index=order - 1,
                member_indices=members,
                walking_distances_m=[0.0] * len(members),
                latitude=float(lat),
                longitude=float(lon),
                label=label,
                source=source,
            )
        )

    if not route_map:
        raise ValueError(
            "Önceki plan dosyasında geçerli rota durağı bulunamadı."
        )

    return [
        route_map[number]
        for number in sorted(route_map)
    ]




def _format_clock(seconds_from_midnight: float) -> str:
    """Saniye cinsinden günü HH:MM biçimine yuvarlar."""
    rounded_minutes = int((float(seconds_from_midnight) + 30) // 60) % (24 * 60)
    return f"{rounded_minutes // 60:02d}:{rounded_minutes % 60:02d}"


def _route_stop_schedule_seconds(
    ordered_matrix_indices: list[int],
    duration_matrix: list[list[float]],
    direction: str,
    wait_seconds_per_stop: int,
) -> tuple[list[float], float]:
    """Durak saatlerini 07:55 fabrika varışı / 17:40 fabrika çıkışına göre hesaplar."""
    if not ordered_matrix_indices:
        anchor = (
            MORNING_FACTORY_ARRIVAL_SECONDS
            if direction == "morning"
            else EVENING_FACTORY_DEPARTURE_SECONDS
        )
        return [], float(anchor)

    if direction == "morning":
        cursor = float(MORNING_FACTORY_ARRIVAL_SECONDS)
        arrivals = [0.0] * len(ordered_matrix_indices)
        for position in range(len(ordered_matrix_indices) - 1, -1, -1):
            current_index = ordered_matrix_indices[position]
            next_index = 0 if position == len(ordered_matrix_indices) - 1 else ordered_matrix_indices[position + 1]
            cursor -= float(duration_matrix[current_index][next_index])
            cursor -= float(wait_seconds_per_stop)
            arrivals[position] = cursor
        return arrivals, float(MORNING_FACTORY_ARRIVAL_SECONDS)

    cursor = float(EVENING_FACTORY_DEPARTURE_SECONDS)
    arrivals: list[float] = []
    previous_index = 0
    for current_index in ordered_matrix_indices:
        cursor += float(duration_matrix[previous_index][current_index])
        arrivals.append(cursor)
        cursor += float(wait_seconds_per_stop)
        previous_index = current_index
    return arrivals, float(EVENING_FACTORY_DEPARTURE_SECONDS)


def materialize_shared_routes(
    allocated_routes: list[list[CommonStop]],
    duration_matrix: list[list[float]],
    distance_matrix: list[list[float]],
    direction: str,
    wait_seconds_per_stop: int,
) -> list[dict]:
    """Ortak durak nesnelerini arayüz ve Excel çıktısının kullandığı yapıya çevirir."""
    shared_routes = []
    for vehicle_no, allocated_stops in enumerate(allocated_routes, start=1):
        stops = []
        for stop in allocated_stops:
            members = list(stop.member_indices)
            walk_by_employee = {
                employee_index: distance
                for employee_index, distance in zip(stop.member_indices, stop.walking_distances_m)
            }
            stops.append(
                {
                    "anchor_matrix_index": stop.matrix_index,
                    "latitude": float(stop.latitude),
                    "longitude": float(stop.longitude),
                    "label": stop.label,
                    "source": stop.source,
                    "member_indices": members,
                    "walk_by_employee": walk_by_employee,
                    "passenger_count": len(members),
                    "max_walk_m": stop.max_walk_m,
                    "average_walk_m": stop.average_walk_m,
                }
            )

        ordered_matrix_indices = [stop["anchor_matrix_index"] for stop in stops]
        schedule_seconds, factory_anchor_seconds = _route_stop_schedule_seconds(
            ordered_matrix_indices,
            duration_matrix,
            direction,
            wait_seconds_per_stop,
        )
        for stop, scheduled_seconds in zip(stops, schedule_seconds):
            stop["scheduled_seconds"] = scheduled_seconds
            stop["scheduled_time"] = _format_clock(scheduled_seconds)
        path_indices = [*ordered_matrix_indices, 0] if direction == "morning" else [0, *ordered_matrix_indices]
        drive_seconds = sum(duration_matrix[a][b] for a, b in zip(path_indices, path_indices[1:]))
        distance_meters = sum(distance_matrix[a][b] for a, b in zip(path_indices, path_indices[1:]))
        all_walks = [
            distance
            for stop in stops
            for distance in stop["walk_by_employee"].values()
        ]
        shared_routes.append(
            {
                "vehicle_no": vehicle_no,
                "direction": direction,
                "factory_time": _format_clock(factory_anchor_seconds),
                "occupancy": sum(stop["passenger_count"] for stop in stops),
                "stops": stops,
                "path_indices": path_indices,
                "distance_km": distance_meters / 1000,
                "drive_minutes": drive_seconds / 60,
                "wait_minutes": len(stops) * wait_seconds_per_stop / 60,
                "total_minutes": drive_seconds / 60 + len(stops) * wait_seconds_per_stop / 60,
                "average_walk_m": sum(all_walks) / len(all_walks) if all_walks else 0,
                "max_walk_m": max(all_walks, default=0),
            }
        )
    return shared_routes


def _allocated_route_total_minutes(
    route: list[CommonStop],
    duration_matrix: list[list[float]],
    direction: str,
    wait_seconds_per_stop: int,
) -> float:
    """Aynı rota grubunun verilen yöndeki sürüş + durak bekleme süresini hesaplar."""
    if not route:
        return 0.0
    matrix_indices = [
        stop.matrix_index if stop.matrix_index is not None else stop.anchor_index + 1
        for stop in route
    ]
    path = [*matrix_indices, 0] if direction == "morning" else [0, *matrix_indices]
    drive_seconds = sum(
        float(duration_matrix[a][b])
        for a, b in zip(path, path[1:])
    )
    return drive_seconds / 60.0 + len(route) * wait_seconds_per_stop / 60.0


def _same_route_directional_times(
    morning_routes: list[list[CommonStop]],
    duration_matrix: list[list[float]],
    wait_seconds_per_stop: int,
) -> tuple[list[float], list[float]]:
    """Aynı çalışan/araç grubu için sabah ve ters akşam sürelerini birlikte hesaplar."""
    morning_times = []
    evening_times = []
    for route in morning_routes:
        if not route:
            continue
        morning_times.append(
            _allocated_route_total_minutes(
                route, duration_matrix, "morning", wait_seconds_per_stop
            )
        )
        evening_times.append(
            _allocated_route_total_minutes(
                list(reversed(route)), duration_matrix, "evening", wait_seconds_per_stop
            )
        )
    return morning_times, evening_times


def _direction_limit_violation_text(
    morning_times: list[float],
    evening_times: list[float],
    max_route_minutes: int,
) -> str:
    violations = []
    for route_no, (morning, evening) in enumerate(zip(morning_times, evening_times), start=1):
        if morning > max_route_minutes + 1e-9 or evening > max_route_minutes + 1e-9:
            violations.append(
                f"Rota {route_no}: sabah {morning:.1f} dk / akşam {evening:.1f} dk"
            )
    return "; ".join(violations)


def build_shared_routes(
    employees: pd.DataFrame,
    factory_coordinates: tuple[float, float],
    max_walk_m: int,
    target_average_walk_m: int,
    direction: str,
    capacity: int,
    mode: str,
    wait_seconds_per_stop: int,
    max_route_minutes: int,
    use_road_network: bool,
    approved_candidates: list[tuple],
    allow_automatic_candidates: bool,
):
    """
    Ortak durakları ve servis rotalarını oluşturur.

    Temel kurallar:
    - Çalışan adresi hiçbir zaman doğrudan servis durağı değildir.
    - Çalışan önce yürüme sınırı içindeki en yakın yüklenen/mevcut durağa atanır.
    - Rota çözüldükten sonra yalnızca güvenli koşullarda, küçük ek yürüyüşle
      aynı rota içindeki yakın duraklar birleştirilebilir.
    - Mevcut durağa yürüyemeyen kişi için güzergâh üzeri veya güzergâha
      minimum sapmalı otomatik aday durak kullanılabilir.
    - Sabit 3 servis seçeneği yalnızca araç sayısını 3'e sabitler;
      eski adres tabanlı referans durak şablonunu kullanmaz.
    - Rota grupları sabah yönünde optimize edilir.
    - Akşam aynı çalışanlar aynı serviste kalır; yalnızca durak sırası ters çevrilir.
    - Azami rota süresi hem sabah hem akşam için kontrol edilir.
    """

    employee_coordinates = list(
        zip(
            employees["Enlem"].astype(float),
            employees["Boylam"].astype(float),
        )
    )

    # ---------------------------------------------------------
    # 1) ADAY DURAKLARI OLUŞTUR
    # ---------------------------------------------------------
    if max_walk_m != 1000:
        raise ValidationError("Azami gerçek yürüme mesafesi sabit 1000 m olmalıdır.")
    if not use_road_network:
        raise ValidationError("Güvenli planlama için OSRM araç yol ağı zorunludur.")
    walking_router = WalkingMatrix()
    all_stops, candidates, minimum_proven, excluded_roads = optimize_major_road_stops(
        employee_coordinates,
        approved_candidates,
        capacity=capacity,
        allow_automatic=allow_automatic_candidates,
        target_average_walk_m=target_average_walk_m,
        walking=walking_router,
    )
    minimum_stop_count = len({(s.latitude, s.longitude) for s in all_stops})
    # The certified optimum concerns a joint stop + walking objective, not
    # a proof of globally minimum number of physical stops.
    joint_objective_proven = minimum_proven
    minimum_proven = False
    # A large common stop may be visited by several buses; no vehicle
    # is assigned more than its own capacity.
    if not all_stops and len(employees):
        raise ValueError(
            "Çalışanlar için yürüme sınırı içinde uygun servis durağı bulunamadı."
        )

    # Son güvenlik kontrolü: ev/adres tabanlı durak sonucu çıkmamalı.
    for stop in all_stops:
        if is_forbidden_home_stop(
            stop.label,
            stop.source,
        ):
            raise ValueError(
                "Optimizasyon çalışan adresini servis durağı olarak üretmeye çalıştı. "
                "Bu sistemde evden servis alımı kapalıdır."
            )

    # ---------------------------------------------------------
    # 3) YOL MATRİSİ
    # ---------------------------------------------------------
    route_coordinates = [
        factory_coordinates,
        *(
            (float(stop.latitude), float(stop.longitude))
            for stop in all_stops
        ),
    ]

    for matrix_index, stop in enumerate(
        all_stops,
        start=1,
    ):
        stop.matrix_index = matrix_index

    (
        duration_matrix,
        distance_matrix,
        matrix_source,
        warnings,
    ) = get_travel_matrices(
        route_coordinates,
        use_road_network=use_road_network,
    )

    if matrix_source != "OSRM yol ağı":
        raise ValidationError(
            "Araç yol ağı doğrulanamadı; kuş uçuşu tahminine geçildiği için plan yayınlanmadı."
        )

    # ---------------------------------------------------------
    # 4) ARAÇ SAYISI
    # ---------------------------------------------------------
    minimum_vehicle_count = math.ceil(
        len(employees) / capacity
    )

    vehicle_count = (
        3
        if mode == "fixed"
        else minimum_vehicle_count
    )

    if vehicle_count < minimum_vehicle_count:
        raise ValueError(
            f"3 araç yetersiz. Bu kapasiteyle en az "
            f"{minimum_vehicle_count} araç gerekir."
        )

    # Aynı servis grubu geliş/dönüşte korunduğu için dağılım
    # her zaman sabah yönünde çözülür.
    planning_direction = "morning"

    last_error: Exception | None = None
    last_direction_violation = ""

    maximum_vehicle_count = max(
        vehicle_count,
        min(
            len(all_stops),
            minimum_vehicle_count + 8,
        ),
    )

    allocated_routes: list[list[CommonStop]] | None = None

    # ---------------------------------------------------------
    # 5) ROTA OPTİMİZASYONU
    # ---------------------------------------------------------
    while vehicle_count <= maximum_vehicle_count:
        try:
            candidate_routes = assign_common_stops_to_routes(
                all_stops,
                route_coordinates,
                vehicle_count,
                capacity,
                duration_matrix,
                planning_direction,
                wait_seconds_per_stop=wait_seconds_per_stop,
                max_route_minutes=max_route_minutes,
                time_limit_seconds=15,
            )

            active_candidate_routes = [
                route
                for route in candidate_routes
                if route
            ]

            if (
                not active_candidate_routes
                and len(employees)
            ):
                raise ValueError(
                    "Optimizasyon aktif bir servis rotası üretemedi."
                )

            morning_times, evening_times = (
                _same_route_directional_times(
                    active_candidate_routes,
                    duration_matrix,
                    wait_seconds_per_stop,
                )
            )

            last_direction_violation = (
                _direction_limit_violation_text(
                    morning_times,
                    evening_times,
                    max_route_minutes,
                )
            )

            if (
                max_route_minutes
                and last_direction_violation
            ):
                if mode == "fixed":
                    raise ValueError(
                        f"Sabit 3 servis ile {max_route_minutes} dk sınırı "
                        "her iki yönde sağlanamıyor. "
                        f"{last_direction_violation}. "
                        "Rota sayısını Otomatik seçin."
                    )

                vehicle_count += 1
                continue

            allocated_routes = active_candidate_routes
            vehicle_count = len(allocated_routes)
            break

        except ValueError as exc:
            last_error = exc

            if mode == "fixed":
                raise

            vehicle_count += 1

    if allocated_routes is None:
        detail = (
            f" Son kontrol: {last_direction_violation}."
            if last_direction_violation
            else ""
        )

        raise ValueError(
            f"Kapasite ve {max_route_minutes} dk sabah/akşam rota süresi "
            f"sınırlarını birlikte sağlayan çözüm bulunamadı.{detail} "
            "Araç kapasitesini veya durak yapısını kontrol edin."
        ) from last_error

    # ---------------------------------------------------------
    # 6) KONTROLLÜ DURAK KONSOLİDASYONU
    # ---------------------------------------------------------
    # Ana OR-Tools rota dağılımını değiştirmiyoruz. Yalnızca AYNI rota içindeki
    # yakın durakları; yürüme sınırı ve iki yönlü süre korunuyorsa birleştiriyoruz.
    # Existing stop consolidation uses air-distance*1.2. It is disabled
    # until a pedestrian-network-safe merge algorithm is available.
    consolidation_meta = {"merged_stop_count": 0, "moved_employee_count": 0, "route_merge_counts": []}
    experience_meta = certify_assigned_routes(
        allocated_routes, employee_coordinates, walking=walking_router, max_walk_m=1000
    )

    # ---------------------------------------------------------
    # 7) SABAH / AKŞAM SÜRELERİ
    # ---------------------------------------------------------
    morning_times, evening_times = (
        _same_route_directional_times(
            allocated_routes,
            duration_matrix,
            wait_seconds_per_stop,
        )
    )

    # ---------------------------------------------------------
    # 8) GÖSTERİLECEK YÖN
    # ---------------------------------------------------------
    output_routes = (
        reverse_routes_for_return(
            allocated_routes
        )
        if direction == "evening"
        else allocated_routes
    )

    shared_routes = materialize_shared_routes(
        output_routes,
        duration_matrix,
        distance_matrix,
        direction,
        wait_seconds_per_stop,
    )

    automatic_sources = {
        "Otomatik ortak nokta",
        "Güzergâh üzeri aday durak",
        "Güzergâha yakın yeni durak",
    }

    if any(
        stop.source in automatic_sources
        for stop in all_stops
    ):
        warnings.append(
            "Otomatik/güzergâh adayı duraklar matematiksel adaydır; "
            "kaldırım, yaya geçidi ve güvenli bekleme alanı sahada "
            "onaylanmalıdır."
        )

    warnings.append(
        "Çalışan adresinden servis alımı yapılmaz. Çalışanlar önce yürüme sınırı "
        "içindeki en yakın yüklenen/mevcut durağa atanır; rota sonrasında yalnızca "
        "kontrollü yakın-durak birleştirmesi uygulanabilir. Mevcut durağa yürüyemeyen "
        "çalışanlar güzergâha uygun otomatik ortak durağa atanır."
    )

    warnings.append(
        "Geliş ve dönüş aynı servis rotasına sabitlenmiştir. Akşam seferinde "
        "çalışanlar başka bir rotaya aktarılmaz; durak sırası sabah rotasının tersidir."
    )

    if consolidation_meta.get("merged_stop_count", 0):
        warnings.append(
            f"Rota dağılımı korunarak {consolidation_meta['merged_stop_count']} yakın durak "
            "kontrollü biçimde birleştirildi. Birleştirmeler yürüme sınırını aşmadı "
            "ve sabah/akşam rota süresini artırmadı."
        )

    warnings.append(
        f"Yaya yol ağı doğrulaması: en uzun yürüyüş {experience_meta['max_walk_m']:.0f} m, "
        f"ortalama {experience_meta['avg_walk_m']:.0f} m (sert sınır 1000 m)."
    )
    if excluded_roads:
        warnings.append(f"Ana yol kontrolünü geçmeyen {len(excluded_roads)} yüklenen durak elendi.")
    if not allow_automatic_candidates:
        warnings.append(
            "Yalnızca saha onaylı duraklar kullanıldı. Overpass ana yol sınıfı servisi "
            "çağrılmadı; araç uygunluğu saha onayına dayanır. Gerçek yaya ağı ve "
            "1.000 m sınırı yine zorunlu olarak kontrol edilir."
        )
    if any(stop.source.startswith("Ana yol adayı") for stop in all_stops):
        warnings.append(
            "OTOMATİK ANA YOL NOKTALARI SADECE ÖNERİDİR. Kaldırım, güvenli bekleme "
            "alanı, geçit, otobüs dönüşü ve araç ölçüleri sahada onaylanmadan uygulanamaz."
        )
    warnings.append(
        "OSRM driving profili büyük servis geometrisini, dönüş çapını ve tonaj/" 
        "yükseklik kısıtlarını kanıtlamaz. Güzergâh sahada servis aracıyla doğrulanmalıdır."
    )
    meta = {
        "vehicle_count": len(allocated_routes),
        "candidate_count": len(candidates),
        "joint_objective_proven": joint_objective_proven,
        "pedestrian_verified": True,
        "road_class_screened": bool(allow_automatic_candidates),
        "max_network_walk_m": experience_meta["max_walk_m"],
        "avg_network_walk_m": experience_meta["avg_walk_m"],
        "minimum_stop_count": minimum_stop_count,
        "minimum_proven": minimum_proven,
        "selected_stop_count": sum(len(route) for route in allocated_routes),
        "pre_consolidation_stop_count": len(all_stops),
        "consolidated_stop_count": consolidation_meta.get("merged_stop_count", 0),
        "consolidated_employee_count": consolidation_meta.get("moved_employee_count", 0),
        "route_consolidation_counts": consolidation_meta.get("route_merge_counts", []),
        "matrix_source": matrix_source,
        "warnings": warnings,
        "planning_mode": "full",
        "same_route_morning_evening": True,
        "hard_route_limit_minutes": max_route_minutes,
        "max_morning_minutes": max(
            morning_times,
            default=0.0,
        ),
        "max_evening_minutes": max(
            evening_times,
            default=0.0,
        ),
    }

    return shared_routes, meta




def build_incremental_shared_routes(
    employees: pd.DataFrame,
    baseline_routes: list[list[CommonStop]],
    factory_coordinates: tuple[float, float],
    max_walk_m: int,
    target_average_walk_m: int,
    direction: str,
    capacity: int,
    mode: str,
    wait_seconds_per_stop: int,
    max_route_minutes: int,
    use_road_network: bool,
    approved_candidates: list[tuple],
    allow_automatic_candidates: bool,
):
    """Mevcut planı günceller; rota grubu geliş/dönüşte aynı kalır."""
    raise ValidationError(
        "Mevcut planı koruma modu gerçek yaya mesafesiyle henüz uyumlu değil. "
        "Sert 1000 m güvenlik şartını korumak için Tam optimizasyonu seçin."
    )

    employee_coordinates = list(
        zip(employees["Enlem"].astype(float), employees["Boylam"].astype(float))
    )
    planning_direction = "morning"
    allocated_routes, duration_matrix, distance_matrix, meta = update_routes_incrementally(
        employee_coordinates=employee_coordinates,
        baseline_routes=baseline_routes,
        factory_coordinates=factory_coordinates,
        approved_candidates=approved_candidates,
        max_walk_m=max_walk_m,
        target_average_walk_m=target_average_walk_m,
        walking_factor=1.20,
        capacity=capacity,
        direction=planning_direction,
        wait_seconds_per_stop=wait_seconds_per_stop,
        max_route_minutes=max_route_minutes,
        mode=mode,
        use_road_network=use_road_network,
        allow_automatic_candidates=allow_automatic_candidates,
    )

    allocated_routes = [route for route in allocated_routes if route]
    morning_times, evening_times = _same_route_directional_times(
        allocated_routes, duration_matrix, wait_seconds_per_stop
    )
    violation = _direction_limit_violation_text(
        morning_times, evening_times, max_route_minutes
    )
    if max_route_minutes and violation:
        raise ValueError(
            f"Mevcut plan korunurken {max_route_minutes} dk sınırı iki yönde birden sağlanamadı: "
            f"{violation}. Bu veri için 'Tam optimizasyon' çalıştırın."
        )

    output_routes = (
        reverse_routes_for_return(allocated_routes)
        if direction == "evening"
        else allocated_routes
    )
    shared_routes = materialize_shared_routes(
        output_routes,
        duration_matrix,
        distance_matrix,
        direction,
        wait_seconds_per_stop,
    )

    meta["vehicle_count"] = len(allocated_routes)
    meta["same_route_morning_evening"] = True
    meta["hard_route_limit_minutes"] = max_route_minutes
    meta["max_morning_minutes"] = max(morning_times, default=0.0)
    meta["max_evening_minutes"] = max(evening_times, default=0.0)
    meta.setdefault("warnings", []).append(
        "Geliş ve dönüş aynı servis rotasına sabitlenmiştir. Akşam seferinde çalışanlar "
        "başka bir rotaya aktarılmaz; durak sırası sabah rotasının tersidir."
    )
    return shared_routes, meta


def result_workbook(shared_routes, employees: pd.DataFrame, capacity: int) -> bytes:
    summary_rows = []
    stop_rows = []
    assignment_rows = []
    for route in shared_routes:
        summary_rows.append(
            {
                "Rota": f"Rota {route['vehicle_no']}",
                "Hedef_Fabrika_Saati": route.get("factory_time", ""),
                "Yolcu": route["occupancy"],
                "Kapasite": capacity,
                "Doluluk_Orani": route["occupancy"] / capacity,
                "Toplam_Durak_Sayisi": len(route["stops"]),
                "Coklu_Ortak_Durak": sum(stop["passenger_count"] > 1 for stop in route["stops"]),
                "Tekil_Durak": sum(stop["passenger_count"] == 1 for stop in route["stops"]),
                "Mesafe_km": round(route["distance_km"], 1),
                "Surus_Suresi_dk": round(route["drive_minutes"]),
                "Bekleme_Suresi_dk": round(route["wait_minutes"]),
                "Toplam_Sure_dk": round(route["total_minutes"]),
                "Ort_Yurume_m": round(route["average_walk_m"]),
                "En_Uzak_Yurume_m": round(route["max_walk_m"]),
            }
        )
        for order, stop in enumerate(route["stops"], start=1):
            stop_rows.append(
                {
                    "Rota": f"Rota {route['vehicle_no']}",
                    "Durak_Sirasi": order,
                    "Tahmini_Saat": stop.get("scheduled_time", ""),
                    "Durak_Adi": professional_stop_name(stop, route["vehicle_no"], order),
                    "Durak_Kaynagi": professional_stop_source(stop["source"]),
                    "Saha_Kontrolu": stop_field_check(stop["source"]),
                    "Durak_Turu": "Ortak" if stop["passenger_count"] > 1 else "Tekil",
                    "Yolcu_Sayisi": stop["passenger_count"],
                    "Enlem": stop["latitude"],
                    "Boylam": stop["longitude"],
                    "En_Uzak_Yurume_m": round(stop["max_walk_m"]),
                }
            )
            for employee_index in stop["member_indices"]:
                row = employees.iloc[employee_index]
                assignment_rows.append(
                    {
                        "Rota": f"Rota {route['vehicle_no']}",
                        "Durak_Sirasi": order,
                        "Tahmini_Saat": stop.get("scheduled_time", ""),
                        "Calisan_ID": row["Calisan_ID"],
                        "Ad_Soyad": row["Ad_Soyad"],
                        "Ev_Adresi": row["Adres"],
                        "Durak_Adi": professional_stop_name(stop, route["vehicle_no"], order),
                        "Durak_Kaynagi": professional_stop_source(stop["source"]),
                        "Saha_Kontrolu": stop_field_check(stop["source"]),
                        "Yurume_Mesafesi_m": round(stop["walk_by_employee"][employee_index]),
                        "Durak_Enlem": stop["latitude"],
                        "Durak_Boylam": stop["longitude"],
                    }
                )
    output = BytesIO()
    with pd.ExcelWriter(output, engine="xlsxwriter") as writer:
        pd.DataFrame(summary_rows).to_excel(writer, sheet_name="Rota_Ozeti", index=False)
        pd.DataFrame(stop_rows).to_excel(writer, sheet_name="Ortak_Duraklar", index=False)
        pd.DataFrame(assignment_rows).to_excel(writer, sheet_name="Personel_Durak_Eslesmesi", index=False)
        workbook = writer.book
        header = workbook.add_format({"bold": True, "font_color": "white", "bg_color": "#285A84"})
        percent = workbook.add_format({"num_format": "0%"})
        frames = (
            ("Rota_Ozeti", pd.DataFrame(summary_rows)),
            ("Ortak_Duraklar", pd.DataFrame(stop_rows)),
            ("Personel_Durak_Eslesmesi", pd.DataFrame(assignment_rows)),
        )
        for sheet_name, frame in frames:
            sheet = writer.sheets[sheet_name]
            for col, name in enumerate(frame.columns):
                sheet.write(0, col, name, header)
                width = min(max(len(name) + 2, *(len(str(v)) + 2 for v in frame[name].head(200))), 42)
                sheet.set_column(col, col, width)
            sheet.freeze_panes(1, 0)
            sheet.autofilter(0, 0, max(len(frame), 1), max(len(frame.columns) - 1, 0))
        writer.sheets["Rota_Ozeti"].set_column("D:D", 15, percent)
        writer.sheets["Rota_Ozeti"].set_column("E:M", 18)
    return output.getvalue()


with st.sidebar:
    st.markdown(
        """
        <div class="sidebar-brand">
            <strong>Planlama Ayarları</strong>
            <span>Optimizasyon kısıtlarını belirleyin</span>
        </div>
        """,
        unsafe_allow_html=True,
    )

    with st.expander("Planlama modeli", expanded=True):
        planning_label = st.selectbox(
            "Planlama yaklaşımı",
            ["Tam optimizasyon"],
            help=(
                "Tam optimizasyon yol ve yaya ağı kontrolleriyle yeni planı oluşturur. "
                "Eski planı koruma modu, gerçek yaya ağına uyarlanana kadar kapalıdır."
            ),
        )
        mode_label = st.selectbox(
            "Rota sayısı",
            ["Otomatik (kapasite + süreye göre)", "Sabit 3 servis"],
            index=0,
        )
        if mode_label.startswith("Sabit 3"):
            st.caption(
                "Bu seçenek yalnızca servis sayısını 3'te sabitler. Duraklar "
                "mevcut/yüklenen duraklar ve güzergâha uygun otomatik adaylar "
                "üzerinden yeniden hesaplanır; çalışan adresi doğrudan durak yapılmaz."
            )
        stop_policy_label = st.selectbox(
            "Durak politikası",
            [
                "Yüklenen durakları kullan; gerekirse yeni aday öner",
                "Yalnızca yüklenen durakları kullan",
            ],
            help=(
                "Önce saha onaylı ana yol durakları değerlendirilir; yetersiz kalırsa "
                "ana yollar üzerinden yeni ortak durak önerilir. Otomatik öneriler uygulama "
                "öncesi saha kontrolüne tabidir. Gerçek yaya yolunda 1.000 m sınırdır."
            ),
        )

    with st.expander("Kapasite ve süre kısıtları", expanded=True):
        capacity = st.number_input(
            "Araç kapasitesi",
            min_value=1,
            max_value=100,
            value=45,
            step=1,
        )
        max_walk_m = 1000
        st.caption("🔒 Gerçek yaya yoluna göre maksimum 1.000 metre (değiştirilemez).")
        target_average_walk_m = min(FIXED_TARGET_AVERAGE_WALK_M, int(max_walk_m))
        max_route_minutes = st.slider(
            "Azami rota süresi",
            min_value=20,
            max_value=100,
            value=70,
            step=5,
            format="%d dk",
            key="max_route_minutes_v8",
            help=(
                "Sürüş + durak bekleme süresidir. Seçtiğiniz değer sabah ve akşam için "
                "kontrol edilir; otomatik mod gerekirse servis sayısını artırır."
            ),
        )
        wait_seconds_per_stop = FIXED_WAIT_SECONDS_PER_STOP

    with st.expander("Sefer ve yol hesabı", expanded=True):
        direction_label = st.selectbox(
            "Sefer yönü",
            [
                "Sabah (07.55 varış): çalışan → fabrika",
                "Akşam (17.40 çıkış): fabrika → çalışan",
            ],
        )
        st.info(
            "🔒 Aynı çalışanlar geliş ve dönüşte aynı servis rotasında kalır. "
            f"Sistem seçtiğiniz {max_route_minutes} dk sınırını hem sabah hem akşam için kontrol eder."
        )
        if max_route_minutes > 70:
            st.warning(
                "Operasyonel hedef 70 dk olarak belirlendi. Daha yüksek bir sınır seçildiğinde "
                "sistem 70 dk üzerindeki çözümleri de kabul edebilir."
            )
        # Road-network distances are mandatory; there is no approximate mode.
        use_road_network = True
        st.caption("🔒 Araç yol ağı + yaya yol ağı zorunludur; yaklaşık mesafe ile plan üretilmez.")
        road_consent = st.checkbox(
            "Koordinatların harita servislerine gönderilmesini onaylıyorum",
            help=(
                "Çalışan ve durak koordinatları araç OSRM ve yaya OSRM sunucularına; "
                "sorgu bölgesinin sınırları Overpass sunucusuna gider. "
                "İsim, sicil ve açık adres gönderilmez."
            ),
        )

    st.markdown(
        """
        <div class="sidebar-note">
            <strong>Çalışma düzeni</strong><br>
            Sabah hedef fabrika varışı 07.55 · Akşam çıkış 17.40 · Durak bekleme süresi 15 sn ·
            Yakın çalışanlar ortak buluşma noktasında eşleştirilir.
        </div>
        """,
        unsafe_allow_html=True,
    )

allow_automatic_candidates = not stop_policy_label.startswith("Yalnızca")
section_header(
    "01",
    "Veri yükleme",
    "Güncel çalışan listesini ve varsa onaylı durak listenizi sisteme ekleyin.",
)
upload_left, upload_right = st.columns(2, gap="large")
with upload_left:
    uploaded = st.file_uploader(
        "Çalışan listesi",
        type=["xlsx", "xls"],
        help="Dosyada çalışanların enlem ve boylam bilgilerinin bulunması gerekir.",
    )
with upload_right:
    approved_stop_file = st.file_uploader(
        "Mevcut / aday durak listesi",
        type=["xlsx", "xls"],
        help=(
            "Durak_Adi, Enlem ve Boylam sütunlarını içeren dosyadır. Saha_Onayi sütununda "
            "Yalnızca Saha_Onayi=Onaylı/Uygun/Evet olan duraklar kullanılır; diğerleri dışlanır."
        ),
    )
previous_plan_file = None
if planning_label.startswith("Mevcut"):
    previous_plan_file = st.file_uploader(
        "Önceki rota sonuç dosyası",
        type=["xlsx"],
        help=(
            "Bir önceki çalıştırmada `Rota sonuçlarını Excel olarak indir` düğmesiyle "
            "kaydettiğiniz servis_rota_sonuclari.xlsx dosyasıdır."
        ),
    )
    if previous_plan_file is None and st.session_state.get("last_plan_bytes") is None:
        st.info(
            "İlk kez kullanıyorsanız önce Tam optimizasyonu çalıştırıp sonucu indirin. "
            "Sonraki çalışan güncellemelerinde bu modu seçin."
        )

if uploaded is None:
    st.info("Başlamak için enlem ve boylam bilgileri dolu olan çalışan listenizi yükleyin.")
    st.stop()

raw_bytes = uploaded.getvalue()
approved_bytes = approved_stop_file.getvalue() if approved_stop_file is not None else b""
previous_bytes = previous_plan_file.getvalue() if previous_plan_file is not None else b""
file_key = hashlib.sha256(
    raw_bytes
    + approved_bytes
    + previous_bytes
    + planning_label.encode("utf-8")
    + stop_policy_label.encode("utf-8")
    + APP_VERSION.encode("utf-8")
).hexdigest()
if st.session_state.get("file_key") != file_key:
    try:
        st.session_state["raw_df"] = pd.read_excel(BytesIO(raw_bytes))
        approved_candidates, approved_stats = read_approved_candidates(approved_stop_file)
        st.session_state["approved_candidates"] = approved_candidates
        st.session_state["approved_candidate_stats"] = approved_stats
        st.session_state["file_key"] = file_key
        st.session_state.pop("result", None)
        st.session_state.pop("shared_routes", None)
    except Exception as exc:
        st.error(f"Excel dosyası okunamadı: {exc}")
        st.stop()

raw_df = st.session_state["raw_df"]
mapping = get_mapping(raw_df)
working = standardize(raw_df, mapping)

if mapping["lat"] is None or mapping["lon"] is None:
    st.error("Excel'de `Enlem` ve `Boylam` sütunları bulunamadı.")
    st.stop()

valid_people = working[working["Aktif_mi"] & working["Servis_Kullaniyor_mu"]].copy()
factory_mask = valid_people["Tip"].apply(is_factory)
factory_rows = valid_people[factory_mask]
employees = valid_people[~factory_mask].reset_index(drop=True)

section_header(
    "02",
    "Veri kontrolü",
    "Rota hesabına alınacak çalışanları, koordinat durumunu ve kapasite ihtiyacını doğrulayın.",
)
approved_count = len(st.session_state.get("approved_candidates", []))
c1, c2, c3, c4 = st.columns(4)
c1.metric("Aktif servis kullanıcısı", len(employees))
c2.metric("Eksik çalışan koordinatı", int(employees[["Enlem", "Boylam"]].isna().any(axis=1).sum()))
c3.metric("Kapasiteye göre minimum", math.ceil(len(employees) / int(capacity)) if len(employees) else 0)
c4.metric("Kullanılabilir durak", approved_count)
candidate_stats = st.session_state.get("approved_candidate_stats", {})
if approved_stop_file is not None:
    st.caption(
        f"{approved_count} saha onaylı durak kullanılabilecek, "
        f"{candidate_stats.get('pending', 0)} onay beklediği için kullanılmayacak, "
        f"{candidate_stats.get('excluded', 0)} reddedilmiş durak dışlanacak. "
        + (
            "Bunlara ek olarak gerektiğinde otomatik güzergâh adayları da değerlendirilecek."
            if allow_automatic_candidates
            else "Bu çalıştırmada bu listenin dışına yeni durak eklenmeyecek."
        )
    )

with st.expander("Çalışan verisini görüntüle", expanded=False):
    st.dataframe(working, width="stretch", hide_index=True)

st.markdown("##### Fabrika bilgisi")
if not factory_rows.empty and factory_rows[["Enlem", "Boylam"]].notna().all(axis=1).any():
    factory = factory_rows[factory_rows[["Enlem", "Boylam"]].notna().all(axis=1)].iloc[0]
    factory_lat = float(factory["Enlem"])
    factory_lon = float(factory["Boylam"])
    factory_address = factory["Adres"] or "Fabrika"
    st.success(f"Excel'den alındı: {factory_address} ({factory_lat:.5f}, {factory_lon:.5f})")
else:
    f1, f2, f3 = st.columns([2, 1, 1])
    factory_address = f1.text_input("Fabrika adresi", value=FACTORY_ADDRESS)
    factory_lat = f2.number_input("Fabrika enlem", value=FACTORY_LAT, format="%.6f")
    factory_lon = f3.number_input("Fabrika boylam", value=FACTORY_LON, format="%.6f")
    st.caption(
        "Koordinatlar VitrA Bozüyük kampüs merkezini yaklaşık gösterir. "
        "Servis araçlarının kullandığı kapının koordinatı biliniyorsa onunla değiştirin."
    )

missing_mask = employees[["Enlem", "Boylam"]].isna().any(axis=1)
if missing_mask.any():
    missing_rows = ", ".join(str(index + 2) for index in employees.index[missing_mask][:20])
    st.error(
        f"{int(missing_mask.sum())} çalışanın Enlem veya Boylam bilgisi eksik. "
        f"Excel satırlarını tamamlayıp dosyayı yeniden yükleyin: {missing_rows}"
    )
else:
    st.success(f"{len(employees)} personelin koordinatı hazır. Rota hesabına geçebilirsiniz.")

ready = not employees.empty and employees[["Enlem", "Boylam"]].notna().all(axis=1).all()
road_ready = not use_road_network or road_consent
incremental_mode = planning_label.startswith("Mevcut")
previous_plan_source = previous_plan_file or st.session_state.get("last_plan_bytes")
incremental_ready = not incremental_mode or (
    previous_plan_source is not None and mapping["id"] is not None
)
approved_ready = allow_automatic_candidates or bool(
    st.session_state.get("approved_candidates", [])
)
section_header(
    "03",
    "Rotaları oluştur",
    "Seçilen ayarlara göre ortak durakları, araç dağılımını ve güzergâh sırasını hesaplayın.",
)
if use_road_network and not road_consent:
    st.info("Gerçek yol güzergâhı için sol menüdeki koordinat paylaşım onayını işaretleyin.")
if incremental_mode and mapping["id"] is None:
    st.error("Mevcut planı korumak için çalışan Excel'inde benzersiz bir sicil/ID sütunu seçilmelidir.")
if not approved_ready:
    st.error("Yalnızca yüklenen duraklar modunda mevcut/adayı durak Excel'i yüklenmelidir.")
button_label = "Mevcut Planı Güncelle" if incremental_mode else "Rotaları Oluştur"
if st.button(
    button_label,
    type="primary",
    disabled=not ready or not road_ready or not incremental_ready or not approved_ready,
    use_container_width=True,
):
    with st.spinner("Ortak duraklar seçiliyor ve rotalar birlikte optimize ediliyor..."):
        try:
            mode = "fixed" if mode_label.startswith("Sabit") else "auto"
            direction = "morning" if direction_label.startswith("Sabah") else "evening"
            common_arguments = {
                "employees": employees,
                "factory_coordinates": (factory_lat, factory_lon),
                "max_walk_m": int(max_walk_m),
                "target_average_walk_m": min(int(target_average_walk_m), int(max_walk_m)),
                "direction": direction,
                "capacity": int(capacity),
                "mode": mode,
                "wait_seconds_per_stop": int(wait_seconds_per_stop),
                "max_route_minutes": int(max_route_minutes),
                "use_road_network": bool(use_road_network),
                "approved_candidates": st.session_state.get("approved_candidates", []),
                "allow_automatic_candidates": allow_automatic_candidates,
            }
            if incremental_mode:
                baseline_routes = read_previous_routes(previous_plan_source, employees)
                shared_routes, result_meta = build_incremental_shared_routes(
                    baseline_routes=baseline_routes,
                    **common_arguments,
                )
            else:
                shared_routes, result_meta = build_shared_routes(**common_arguments)
            st.session_state["result"] = result_meta
            st.session_state["shared_routes"] = shared_routes
            st.session_state["result_employees"] = employees.copy()
            st.session_state["result_factory"] = (factory_lat, factory_lon, factory_address)
            st.session_state["result_direction"] = direction_label
            st.session_state["result_capacity"] = int(capacity)
            st.session_state["result_max_walk_m"] = int(max_walk_m)
            st.session_state["result_target_average_walk_m"] = min(
                int(target_average_walk_m), int(max_walk_m)
            )
            st.session_state["result_wait_seconds"] = int(wait_seconds_per_stop)
            st.session_state["result_max_route_minutes"] = int(max_route_minutes)
            st.session_state["result_mode"] = mode
            st.session_state["result_planning_mode"] = "incremental" if incremental_mode else "full"
            st.session_state["result_allow_automatic_candidates"] = allow_automatic_candidates
        except Exception as exc:
            st.session_state.pop("result", None)
            st.session_state.pop("shared_routes", None)
            st.error(str(exc))

result = st.session_state.get("result")
if result is None:
    if not ready:
        st.error("Optimizasyonu çalıştırmak için tüm aktif çalışanların enlem ve boylamı bulunmalı.")
    st.stop()

employees = st.session_state["result_employees"]
factory_lat, factory_lon, factory_address = st.session_state["result_factory"]
result_direction = st.session_state["result_direction"]
result_capacity = st.session_state["result_capacity"]
result_max_walk_m = st.session_state.get("result_max_walk_m", 500)
direction = "morning" if result_direction.startswith("Sabah") else "evening"
shared_routes = st.session_state.get("shared_routes")
if shared_routes is None:
    st.error("Rota sonucu bulunamadı. Optimizasyonu yeniden çalıştırın.")
    st.stop()
optimized_vehicle_count = int(result.get("vehicle_count", 0))
result_wait_seconds = st.session_state.get("result_wait_seconds", 15)
result_max_route_minutes = st.session_state.get("result_max_route_minutes", 70)
result_target_average_walk_m = st.session_state.get(
    "result_target_average_walk_m", FIXED_TARGET_AVERAGE_WALK_M
)

section_header(
    "04",
    "Optimizasyon sonucu",
    "Önerilen servis planının temel performans göstergelerini ve güzergâh detaylarını inceleyin.",
)
if result.get("pedestrian_verified"):
    st.success(
        f"Gerçek yaya yolunda doğrulandı: en uzun yürüyüş "
        f"{result.get('max_network_walk_m', 0):.0f} m, "
        f"ortalama {result.get('avg_network_walk_m', 0):.0f} m. "
        "Hiçbir çalışan 1.000 m üzerinde yürümeye atanmamıştır."
    )
for warning in result.get("warnings", []):
    if "Yaya yol ağı doğrulaması:" in warning:
        continue  # already displayed in result summary
    if "OTOMATİK ANA YOL" in warning or "OSRM driving profili" in warning:
        st.warning(warning)
    else:
        st.caption("ℹ️ " + warning)

if result.get("same_route_morning_evening"):
    max_morning = float(result.get("max_morning_minutes", 0.0))
    max_evening = float(result.get("max_evening_minutes", 0.0))
    hard_limit = int(result.get("hard_route_limit_minutes", result_max_route_minutes))
    if max(max_morning, max_evening) <= hard_limit + 1e-9:
        st.success(
            f"Süre kontrolü tamam: en uzun sabah rotası {max_morning:.0f} dk, "
            f"en uzun akşam rotası {max_evening:.0f} dk — her ikisi de ≤ {hard_limit} dk."
        )

nonempty_routes = [route for route in shared_routes if route["occupancy"]]
# Ekrandaki servis sayısı meta verideki nominal araç sayısı değil, gerçekten kullanılan rota sayısıdır.
optimized_vehicle_count = len(nonempty_routes)
result["vehicle_count"] = optimized_vehicle_count
avg_fill = sum(route["occupancy"] for route in nonempty_routes) / (len(nonempty_routes) * result_capacity) if nonempty_routes else 0
total_stop_count = sum(len(route["stops"]) for route in nonempty_routes)
automatic_stop_sources = {
    "Otomatik ortak nokta",
    "Güzergâh üzeri aday durak",
    "Güzergâha yakın yeni durak",
    "Ana yol adayı (saha kontrolü gerekli)",
}
automatic_stop_count = sum(
    stop.get("source") in automatic_stop_sources
    for route in nonempty_routes
    for stop in route["stops"]
)
automatic_single_stop_count = sum(
    stop.get("source") in automatic_stop_sources
    and stop.get("passenger_count", 0) == 1
    for route in nonempty_routes
    for stop in route["stops"]
)
multi_stop_count = sum(
    stop["passenger_count"] > 1
    for route in nonempty_routes
    for stop in route["stops"]
)
single_stop_count = total_stop_count - multi_stop_count
all_walks = [
    distance
    for route in nonempty_routes
    for stop in route["stops"]
    for distance in stop["walk_by_employee"].values()
]
average_walk = sum(all_walks) / len(all_walks) if all_walks else 0
maximum_walk = max(all_walks, default=0)
total_distance = sum(route["distance_km"] for route in nonempty_routes)
longest_route = max((route["total_minutes"] for route in nonempty_routes), default=0)
r1, r2, r3, r4 = st.columns(4)
r1.metric("Önerilen servis", optimized_vehicle_count)
r2.metric("Toplam çalışan", len(employees))
r3.metric("Toplam durak", total_stop_count)
r4.metric("Ortalama doluluk", f"%{avg_fill * 100:.0f}")
s1, s2, s3, s4 = st.columns(4)
s1.metric("Ortalama yürüme", f"{average_walk:.0f} m")
s2.metric("En uzun yürüme", f"{maximum_walk:.0f} m")
s3.metric("Toplam rota mesafesi", f"{total_distance:.1f} km")
s4.metric("En uzun rota", f"{longest_route:.0f} dk")

if longest_route <= 70 + 1e-9 and maximum_walk <= result_max_walk_m + 1e-9:
    st.success(
        f"Operasyonel kontrol uygun: en uzun rota {longest_route:.0f} dk, "
        f"en uzun yürüme {maximum_walk:.0f} m. "
        "Yeni/otomatik duraklar saha kontrolünden sonra kesinleştirilmelidir."
    )
elif longest_route > 70 + 1e-9:
    st.warning(
        f"En uzun rota {longest_route:.0f} dk. Operasyonel 70 dk hedefinin üzerindedir."
    )

with st.expander("Teknik optimizasyon ayrıntıları", expanded=False):
    t1, t2, t3, t4 = st.columns(4)
    if result.get("planning_mode") == "incremental":
        t1.metric("Eşleşmesi korunan", result.get("preserved_employee_count", 0))
        t2.metric("Mevcut durağa eklenen", result.get("added_to_existing_count", 0))
        t3.metric("Yeni durak", result.get("new_stop_count", 0))
        t4.metric("Yeni rota", result.get("added_route_count", 0))
    else:
        t1.metric("Değerlendirilen aday durak", result.get("candidate_count", 0))
        t2.metric(
            "Seçilen fiziksel durak",
            result.get("minimum_stop_count", 0),
        )
        t3.metric("Çoklu ortak durak", multi_stop_count)
        t4.metric("Tekil durak", single_stop_count)
if automatic_stop_count:
    st.caption(
        f"{automatic_stop_count} otomatik güzergâh durağı önerildi "
        f"({automatic_single_stop_count} tanesi tekil). "
        "Yeni model tekil otomatik durakları ve farklı koridorlar arası gereksiz "
        "geçişleri cezalandırır; kesinleştirilmeden önce saha uygunluğu yine kontrol edilmelidir."
    )

if result.get("planning_mode") != "incremental" and result.get("consolidated_stop_count", 0):
    before_stops = int(result.get("pre_consolidation_stop_count", total_stop_count))
    merged_stops = int(result.get("consolidated_stop_count", 0))
    st.success(
        f"Kontrollü konsolidasyon: {before_stops} başlangıç durağından "
        f"{merged_stops} durak güvenli biçimde birleştirildi; sonuç {total_stop_count} durak."
    )
if result.get("planning_mode") == "incremental":
    st.success(
        f"Mevcut plan korundu: {result.get('preserved_employee_count', 0)} çalışan eski durağında kaldı; "
        f"{result.get('added_to_existing_count', 0)} yeni/adresi değişen çalışan mevcut durağa eklendi; "
        f"{result.get('new_stop_count', 0)} yeni durak ve {result.get('added_route_count', 0)} yeni rota açıldı."
    )
    st.caption(
        f"Artımlı güncelleme: önceki duraklar ve sıraları korunarak {result_max_walk_m} m yürüme, "
        f"{result_capacity} kişi kapasite, durak başına {result_wait_seconds} sn bekleme ve "
        f"{result_max_route_minutes} dk rota sınırı uygulandı. Yeni otomatik noktalar saha onayı gerektirir."
    )
else:
    st.caption(
        f"Hesaplama: {result_max_walk_m} m azami yürüyüş, "
        f"{result_target_average_walk_m} m sabit konfor hedefi, "
        f"durak başına {result_wait_seconds} sn bekleme ve "
        f"{result_max_route_minutes} dk azami rota süresi."
    )

palette = [
    [40, 90, 132], [213, 94, 0], [0, 140, 120], [163, 75, 148],
    [230, 159, 0], [86, 180, 233], [204, 121, 167], [0, 114, 178],
]
path_data = []
point_data = [
    {
        "lon": factory_lon,
        "lat": factory_lat,
        "label": "Fabrika",
        "stop_no": "F",
        "color": [196, 44, 44],
        "radius": 150,
    }
]

for route in nonempty_routes:
    color = palette[(route["vehicle_no"] - 1) % len(palette)]
    ordered_coordinates = []
    stop_by_matrix_index = {stop["anchor_matrix_index"]: stop for stop in route["stops"]}
    stop_number_by_index = {
        stop["anchor_matrix_index"]: order for order, stop in enumerate(route["stops"], start=1)
    }
    for matrix_index in route["path_indices"]:
        if matrix_index == 0:
            ordered_coordinates.append((factory_lat, factory_lon))
        else:
            stop = stop_by_matrix_index[matrix_index]
            ordered_coordinates.append((stop["latitude"], stop["longitude"]))
            stop_type = "Ortak" if stop["passenger_count"] > 1 else "Tekil"
            stop_order = stop_number_by_index[matrix_index]
            display_name = professional_stop_name(
                stop,
                route["vehicle_no"],
                stop_order,
            )
            point_data.append(
                {
                    "lon": stop["longitude"], "lat": stop["latitude"],
                    "label": (
                        f"Rota {route['vehicle_no']} · {stop_type} Durak {stop_order} · "
                        f"{stop['passenger_count']} yolcu · {display_name}"
                    ),
                    "stop_no": str(stop_order),
                    "color": color,
                    "radius": 95 + stop["passenger_count"] * 10,
                }
            )
    try:
        geometry = fetch_osrm_geometry(ordered_coordinates) if result["matrix_source"].startswith("OSRM") else []
    except Exception:
        geometry = []
    if not geometry:
        geometry = [[lon, lat] for lat, lon in ordered_coordinates]
    path_data.append({"path": geometry, "color": color, "route": f"Rota {route['vehicle_no']}"})

layers = [
    pdk.Layer("PathLayer", path_data, get_path="path", get_color="color", get_width=6, width_min_pixels=3, pickable=True),
    pdk.Layer(
        "ScatterplotLayer", point_data, get_position="[lon, lat]", get_fill_color="color",
        get_radius="radius", radius_min_pixels=5, radius_max_pixels=13, pickable=True,
    ),
    pdk.Layer(
        "TextLayer",
        point_data,
        get_position="[lon, lat]",
        get_text="stop_no",
        get_color=[255, 255, 255],
        get_size=11,
        get_alignment_baseline="center",
        get_text_anchor="middle",
        pickable=False,
    ),
]
all_lats = [factory_lat, *employees["Enlem"].astype(float).tolist()]
all_lons = [factory_lon, *employees["Boylam"].astype(float).tolist()]
view_state = pdk.ViewState(
    latitude=(min(all_lats) + max(all_lats)) / 2,
    longitude=(min(all_lons) + max(all_lons)) / 2,
    zoom=9.8,
    pitch=0,
)
deck = pdk.Deck(
    layers=layers,
    initial_view_state=view_state,
    map_style="https://basemaps.cartocdn.com/gl/positron-gl-style/style.json",
    tooltip={"text": "{label}"},
)
st.markdown("#### Rota haritası")
st.caption("Rota çizgilerinin ve numaralı durakların ayrıntılarını görmek için harita üzerinde gezinin.")
st.pydeck_chart(deck, width="stretch")

st.markdown("#### Rota detayları")
st.caption("Sabah durak saatleri, fabrikaya 07:55 varış hedefinden OSRM segment süreleri ve 15 sn/durak bekleme ile geriye doğru hesaplanır.")
for route in nonempty_routes:
    route_fill = route["occupancy"] / result_capacity if result_capacity else 0
    st.markdown(
        f"""
        <div class="route-card">
            <div class="route-card-head">
                <div class="route-title">Rota {route['vehicle_no']}</div>
                <div class="route-occupancy">
                    {route['occupancy']} / {result_capacity} yolcu · %{route_fill * 100:.0f} doluluk
                </div>
            </div>
            <div class="route-stat-grid">
                <div class="route-stat"><strong>{len(route['stops'])}</strong><span>Durak</span></div>
                <div class="route-stat"><strong>{route['distance_km']:.1f} km</strong><span>Mesafe</span></div>
                <div class="route-stat"><strong>{route['drive_minutes']:.0f} dk</strong><span>Sürüş</span></div>
                <div class="route-stat"><strong>{route['total_minutes']:.0f} dk</strong><span>Toplam süre</span></div>
                <div class="route-stat"><strong>{route.get('factory_time', '')}</strong><span>{'Fabrika varış' if result_direction.startswith('Sabah') else 'Fabrika çıkış'}</span></div>
                <div class="route-stat"><strong>{route['average_walk_m']:.0f} m</strong><span>Ort. yürüme</span></div>
            </div>
        </div>
        """,
        unsafe_allow_html=True,
    )
    stop_rows = []
    if not result_direction.startswith("Sabah"):
        stop_rows.append(
            {
                "Durak": "Başlangıç",
                "Tahmini Saat": route.get("factory_time", "17:40"),
                "Durak Türü": "Fabrika",
                "Kaynak": "Sabit",
                "Konum": factory_address,
                "Yolcu": None,
                "En Uzak Yürüme": "-",
                "Saha Kontrolü": "-",
                "Bu Durağa Gelecekler": "-",
            }
        )
    for order, stop in enumerate(route["stops"], start=1):
        passenger_names = ", ".join(
            str(employees.iloc[index]["Ad_Soyad"] or employees.iloc[index]["Calisan_ID"])
            for index in stop["member_indices"]
        )
        stop_rows.append(
            {
                "Durak": str(order),
                "Tahmini Saat": stop.get("scheduled_time", ""),
                "Durak Türü": "Ortak" if stop["passenger_count"] > 1 else "Tekil",
                "Kaynak": professional_stop_source(stop["source"]),
                "Konum": professional_stop_name(
                    stop,
                    route["vehicle_no"],
                    order,
                ),
                "Yolcu": stop["passenger_count"],
                "En Uzak Yürüme": f"{stop['max_walk_m']:.0f} m",
                "Saha Kontrolü": stop_field_check(stop["source"]),
                "Bu Durağa Gelecekler": passenger_names,
            }
        )
    if result_direction.startswith("Sabah"):
        stop_rows.append(
            {
                "Durak": "Varış",
                "Tahmini Saat": route.get("factory_time", "07:55"),
                "Durak Türü": "Fabrika",
                "Kaynak": "Sabit",
                "Konum": factory_address,
                "Yolcu": None,
                "En Uzak Yürüme": "-",
                "Saha Kontrolü": "-",
                "Bu Durağa Gelecekler": "-",
            }
        )
    with st.expander(f"Rota {route['vehicle_no']} · Durak ve yolcu listesini görüntüle"):
        st.dataframe(pd.DataFrame(stop_rows), width="stretch", hide_index=True)

export_bytes = result_workbook(shared_routes, employees, result_capacity)
st.session_state["last_plan_bytes"] = export_bytes
st.markdown("#### Raporlama")
st.caption("Rota özetini, durak sıralamasını ve çalışan–durak eşleşmelerini tek Excel dosyasında indirin.")
st.download_button(
    "Rota Sonuçlarını Excel Olarak İndir",
    data=export_bytes,
    file_name="servis_rota_sonuclari.xlsx",
    mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
    type="primary",
    use_container_width=True,
)
