# Servis Optimizasyonu V2 — HTTP 502 düzeltmesi

Yeni `ceydaariisoy/servis-optimizasyon-v2` GitHub deposunda **app.py** ve **route_safety.py** dosyalarını bu klasördeki sürümlerle değiştirin. `core.py` ve `requirements.txt` değişmeyecek. Eski `servis-optimizasyon-web` deposuna dokunmayın.

## Ne değişti?

- 502/503/504/429/500 HTTP durumlarında kısa beklemeyle toplam en fazla 3 deneme; dış servise sürekli istek gönderilmez.
- Hata mesajı artık *Overpass ana yol sorgusu* ve *OSRM yaya yolu sorgusu* arasında ayrım yapar.
- **Yalnızca yüklenen durakları kullan** seçiliyse ve duraklar `Saha_Onayi = Onaylı` ile yüklenmişse Overpass çağrısı atlanır. Bu modda OSM ana yol sınıfı doğrulanmış gibi gösterilmez: otobüs durak güvenliği saha ekibi tarafından gerçekten doğrulanmış olmalıdır.
- Yaya OSRM servisi her durumda gereklidir; **gerçek yürüme mesafesi 1.000 m'yi aşarsa sistem rota üretmez.** Mesafe verisi alınamıyorsa asla kuş uçuşuna düşmez.
- Otomatik ana yol durakları isteniyorsa Overpass gerekebilir. Her iki halka açık servis de 502 verebilir; ağır kullanıma uygun değildir.

## GitHub'a yükleme

1. `ceydaariisoy/servis-optimizasyon-v2` → `Code` → `Add file` → `Upload files`.
2. Bu klasördeki **app.py** ve **route_safety.py** dosyalarını yükleyin.
3. `Commit changes` seçin. Streamlit güncel GitHub sürümünü yeniden kurmalıdır, gerekirse `Manage app` → `Reboot app`.
4. Denemek için sol menüde **Durak politikası → Yalnızca yüklenen durakları kullan** seçin. Bu, ana yol harita servisine ihtiyaç duymadan mevcut saha onaylı durakların kapsamasını test eder. Tüm çalışanlara 1.000 m gerçek yürüme sınırında erişilebilir durak yoksa sistem dürüstçe uyaracaktır.
5. Otomatik yeni durak önerileri için ilk seçeneğe dönün; büyük bölgelerde Overpass'a yönelik ağır sorgular tekrar hata verebilir. Kalıcı çözüm şirket onaylı yol verisi ve kuruma ait / sözleşmeli yaya yönlendirme altyapısı kullanmaktır.

## Güvenlik notu

Kurumsal çalışan koordinatlarını halka açık yaya yönlendirme sunucularına gönderme konusunda gerekli izinler yoksa testleri anonim/veri maskelemesi yapılmış örneklerle gerçekleştirin. `Public` GitHub deposuna gerçek çalışan adresi, koordinatı veya kimlik bilgisi içeren Excel dosyaları yüklemeyin.

## Testler

Çevrimdışı 18 birim testi geçti. Canlı Streamlit/Overpass/OSRM sunucusu ve gerçek çalışan verileriyle uçtan uca doğrulama yapılmadı. 502 kodla tamamen giderilebilecek bir hata değildir; dış hizmet veremiyorsa planlama yine güvenli biçimde durabilir.
