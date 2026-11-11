# STPM32_UART

[English](README.md) | **Türkçe**

**ESP32** (Arduino çekirdeği / PlatformIO) için **ST STPM32** tek fazlı enerji ölçüm entegresinin UART sürücüsü.

- RMS gerilim ve akım; aktif, temel, reaktif ve görünür güç; enerji sayaçları; şebeke frekansı
- Otomatik baud hızı bulma (STPM32 enerjili kalırken ESP32 resetlense bile bağlantı kurulur) ve CRC kontrollü çerçeveler
- Ayarlar NVS'te saklanır, her açılışta güvenli sırayla yeniden yazılır
- Referans sayaca göre kalibrasyon (CHV1 / CHC1)
- Gerilim düşmesi (sag), yükselmesi (swell) ve aşırı akım algılaması, **INT1** pinine yönlendirilir
- Thread-safe: tüm public fonksiyonlar FreeRTOS recursive mutex ile korunur
- `begin()` sonrasında dinamik bellek kullanılmaz

> ⚠️ STPM32 kartları **şebeke gerilimine** bağlıdır. Ne yaptığınızı bilmiyorsanız çalışmayın. Enerjili karta bilgisayar bağlarken izolasyon (izoleli USB / besleme) kullanın.

## Donanım

| STPM32 | ESP32 |
| --- | --- |
| TXD | RX olabilen herhangi bir pin (ör. GPIO16) |
| RXD | TX olabilen herhangi bir pin (ör. GPIO17) |
| INT1 (isteğe bağlı) | herhangi bir giriş pini |
| GND | GND |

STPM32, **EN yükselirken SCS HIGH ise** UART modunda açılır (datasheet, "Communication peripheral"). INT1 **aktif-HIGH**'tır; etkin bir olay durum register'ında latch'li kaldığı sürece HIGH kalır.

Yalnızca birincil kanal (V1 / C1) kullanılır. STPM32'de tamper modülü olmadığı için `setTamperProtection()` her zaman `false` döner.

## Kurulum

**Arduino IDE:** Library Manager → `STPM32_UART` ara → Install.

**PlatformIO** (`platformio.ini`):

```ini
lib_deps = mustafabagr/STPM32_UART
```

**Elle:** depoyu ZIP olarak indir, *Sketch → Include Library → Add .ZIP Library* ile ekle.

## Hızlı başlangıç

```cpp
#include <STPM32_UART.h>

HardwareSerial StpmSerial(1);
STPMClass stpm(StpmSerial);

void setup() {
  Serial.begin(115200);
  stpm.setScale(0.031049f, 0.001341f);   // donanımına ait V/LSB ve x16 kazançta A/LSB
  stpm.begin(16, 17);                     // RX pini, TX pini (hedef hız varsayılan 9600)
}

void loop() {
  StpmSnapshot s;
  if (stpm.readSnapshot(s)) {
    Serial.printf("%.2f V  %.4f A  %.2f W  %.2f Hz\n", s.vRms, s.iRms, s.pActive, s.lineHz);
  }
  delay(1000);
}
```

## Ölçekleme

Register değerlerinin volt ve ampere çevrilmesi gerilim bölücüne ve akım sensörüne bağlıdır. Varsayılan değerler (`STPM_DEFAULT_V_LSB`, `STPM_DEFAULT_I_LSB_X16`) referans tasarımda (810 kΩ bölücü, 500 µΩ şönt) ölçülmüştür; başka donanımda tutmaz.

1. Varsayılanlarla ya da datasheet'ten ("RMS values" bölümü) hesapladığın değerlerle başla.
2. Bilinen bir gerilim ve akım uygula, kütüphaneyle oku ve düzelt:
   `yeniLsb = eskiLsb × gerçekDeğer / ölçülenDeğer`.
3. Sonucu `begin()`'den **önce** `setScale(vLsb, iLsbAtGain16)` ile ver.
4. İnce ayar için entegrenin kendi kalibrasyonunu kullan (`setVoltageTrim()` / `setCurrentTrim()`); `Calibration` örneğine bak.

Güç LSB'si `16 × LSB_V × LSB_I` olarak hesaplanır; akım LSB'si akım kazancına (`setCurrentGain()`) göre otomatik güncellenir.

## Örnekler

| Örnek | Gösterdiği |
| --- | --- |
| `BasicReadout` | Saniyede bir tüm ölçümler, enerji biriktirme, bağlantı kurtarma |
| `Protection` | INT1 üzerinden sag / swell / aşırı akım; durum register'ını okuyup seçerek temizleme |
| `Calibration` | Seri monitörden referans sayaca göre gerilim ve akım kalibrasyonu |

## API özeti

| Fonksiyon | Açıklama |
| --- | --- |
| `setScale(vLsb, iLsbAtGain16)` | Donanım ölçek katsayıları; `begin()`'den önce çağır |
| `begin(rx, tx, targetBaud = 9600, nvsNamespace = "stpm_cfg")` | UART'ı başlatır, ayarları NVS'ten yükler, bağlanır ve uygular |
| `recoverLink()` | Hatalardan sonra bağlantıyı yeniden kurar. 0 = başarısız, 1 = tamam, 2 = tamam ve entegre resetlenmişti (ayarlar yeniden yazıldı) |
| `readSnapshot(StpmSnapshot&)` | Tüm ölçümler tek kilit altında |
| `readVoltageAndCurrent(v, i)`, `readActivePower(p)`, `readApparentPower(s)` | Tekil okumalar |
| `energyLsbWh()` | Enerji register'larında bir sayımın Wh karşılığı |
| `setCurrentGain(kod)` | 0: x2, 1: x4, 2: x8, 3: x16 (aşırı akım eşiğini yeniden hesaplar) |
| `setVoltageTrim(beklenen, ölçülen)`, `setCurrentTrim(beklenen, ölçülen)` | Entegre kalibrasyonu (NVS'e kaydedilir) |
| `setVoltageProtection(aktif, sagV, swellV)`, `setCurrentProtection(aktif, maxA)` | Eşikler ve INT1 maskesi (NVS'e kaydedilir) |
| `getProtectionSettings(...)` | Mevcut eşikler ve etkinlik durumu |
| `readStatus(sr1)`, `clearStatus(maske)` | Latch'li durum register'ı (`STPM_IRQ::*` bit maskeleri) |
| `readLiveEvents(ev1)`, `clearSagSwellHistory()` | Canlı olaylar ve sag/swell geçmişi |
| `readRegister32(adres, &v)`, `writeRegister32(adres, v)` | Ham register erişimi (register union'ları başlık dosyasında) |

### Notlar

- **Değerler ham.** Anti-creep, güç faktörü ya da enerji politikası uygulanmaz; bunları uygulamanda yap.
- **Enerji sayaçları taşar** (mod 2³²). Okumalar arasındaki işaretli farkı biriktir (`BasicReadout` örneği) ve en yüksek güçte taşmadan önce okuyacak sıklıkta oku.
- **NVS:** birden fazla entegre sürüyorsan her `STPMClass` örneğine ayrı `nvsNamespace` ver.
- **Koruma sırası:** maske kapat → eski durumu temizle → eşiği yaz → bekle → maske aç. Koruma açıldığı anda gerçekten var olan bir durum INT1'i doğru şekilde tetikler.

## Lisans

[MIT](LICENSE)
