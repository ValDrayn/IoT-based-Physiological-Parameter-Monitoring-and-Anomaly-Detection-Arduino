// ============================================================
// PHYSIO MONITOR
// ESP32 + MAX30102 + DS18B20 + OLED SSD1306 + Buzzer + MQTT
//
// Fungsi utama program:
// 1. Membaca detak jantung (HR) dan saturasi oksigen (SpO2)
//    dari sensor MAX30102.
// 2. Membaca suhu tubuh dari sensor DS18B20.
// 3. Menampilkan hasil pengukuran pada layar OLED.
// 4. Membunyikan buzzer jika terdeteksi kondisi anomali.
// 5. Mengirim data ke broker MQTT dalam format JSON
//    setiap 1 detik.
// ============================================================


// ============================================================
// LIBRARY
// Library yang dibutuhkan oleh program.
// ============================================================

// Library untuk koneksi WiFi bawaan ESP32
#include <WiFi.h>

// Library client MQTT untuk publish data ke broker
#include <PubSubClient.h>

// Library komunikasi I2C
#include <Wire.h>

// Library driver sensor MAX30102 / MAX30105
#include <MAX30105.h>

// Berisi fungsi checkForBeat() untuk mendeteksi detak jantung
#include "heartRate.h"

// Berisi algoritma Maxim untuk menghitung SpO2
#include "spo2_algorithm.h"

// Library grafis dan driver layar OLED SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Library komunikasi OneWire dan driver sensor suhu DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>


// ============================================================
// WIFI
// Kredensial jaringan WiFi yang digunakan ESP32.
// ============================================================

// Nama jaringan WiFi (SSID)
const char* WIFI_SSID = "iQOO 15R";

// Password jaringan WiFi
const char* WIFI_PASS = "password";


// ============================================================
// MQTT
// Konfigurasi broker dan topik MQTT untuk pengiriman data.
// ============================================================

// Alamat broker MQTT publik dan port standar (tanpa TLS)
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;

// Topik tujuan publish data sensor
const char* MQTT_TOPIC = "skripsi/physiomonitor/data";

// Client TCP yang menjadi transport untuk MQTT
WiFiClient espClient;

// Client MQTT yang berjalan di atas koneksi WiFi
PubSubClient mqttClient(espClient);


// ============================================================
// PIN
// Definisi pin GPIO ESP32 untuk setiap perangkat.
// ============================================================

// MAX30102 (jalur I2C utama / bus 0)
#define MAX_SDA 21
#define MAX_SCL 22

// DS18B20 (jalur data OneWire)
#define DS18B20_PIN 4

// OLED (jalur I2C kedua / bus 1, terpisah dari MAX30102)
#define OLED_SDA 16
#define OLED_SCL 17

// Buzzer
#define BUZZER_PIN 25

// Buzzer bersifat active-low:
// LOW  = buzzer bunyi
// HIGH = buzzer diam
#define BUZZER_NYALA LOW
#define BUZZER_MATI HIGH


// ============================================================
// OLED
// Inisialisasi objek layar OLED.
// ============================================================

// Bus I2C kedua (nomor 1) khusus untuk OLED agar tidak
// bentrok dengan bus I2C milik MAX30102
TwoWire oledWire = TwoWire(1);

// Layar SSD1306 resolusi 128x64, memakai bus oledWire,
// pin reset -1 berarti tidak memakai pin reset
Adafruit_SSD1306 display(
  128,
  64,
  &oledWire,
  -1
);


// ============================================================
// SENSOR OBJECT
// Objek untuk masing-masing sensor.
// ============================================================

// Objek sensor MAX30102 (HR dan SpO2)
MAX30105 particleSensor;

// Objek bus OneWire dan sensor suhu DS18B20
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);


// ============================================================
// MAX30102 CONFIG
// Parameter konfigurasi sensor MAX30102.
// ============================================================
//
// Konfigurasi ini sengaja mengikuti kode HR kamu yang sudah
// terbukti stabil.
//

// Kecerahan LED (0x00 - 0xFF), 0x3F = sekitar 25%
const byte LED_BRIGHTNESS = 0x3F;

// Jumlah sampel yang dirata-ratakan sebelum masuk FIFO
const byte SAMPLE_AVERAGE = 4;

// Mode LED: 2 = Red + IR (dibutuhkan untuk SpO2)
const byte LED_MODE = 2;

// Laju sampling internal sensor (sampel/detik)
const int SAMPLE_RATE = 100;

// Lebar pulsa LED (411 us = resolusi ADC 18 bit)
const int PULSE_WIDTH = 411;

// Rentang ADC sensor (nA)
const int ADC_RANGE = 4096;


// ============================================================
// IMPORTANT
// Penjelasan laju sampel efektif.
// ============================================================
//
// Karena:
// sample rate = 100 Hz
// averaging   = 4
//
// FIFO output efektif:
// 100 / 4 = 25 samples/second
//
// Ini sesuai dengan FreqS = 25 pada spo2_algorithm.h

// Laju sampel efektif yang keluar dari FIFO (Hz).
// Dipakai sebagai dasar perhitungan BPM.
const float FIFO_SAMPLE_RATE = 25.0;


// ============================================================
// FINGER DETECTION
// Variabel untuk mendeteksi apakah jari menempel di sensor.
// ============================================================

// Batas nilai IR minimum agar jari dianggap menempel
const long FINGER_THRESHOLD = 12000;

// Jika selama 2000 ms tidak ada nilai IR di atas threshold,
// jari dianggap sudah dilepas
const unsigned long FINGER_TIMEOUT = 2000;

// Waktu terakhir jari terdeteksi
unsigned long lastFingerDetected = 0;

// Status keberadaan jari saat ini
bool fingerPresent = false;


// ============================================================
// HEART RATE
// Variabel untuk perhitungan detak jantung.
// ============================================================

// BPM mentah dari satu interval antar-detak
float bpmRaw = 0;

// BPM hasil rata-rata (moving average), ini yang ditampilkan
float bpmFiltered = 0;

// Total sampel FIFO yang sudah dibaca (dipakai sebagai "waktu")
unsigned long totalSamples = 0;

// Nomor sampel saat detak terakhir terdeteksi
unsigned long lastBeatSample = 0;

// Penanda bahwa detak pertama sudah ditemukan
// (detak pertama hanya dipakai sebagai referensi)
bool firstBeatDetected = false;


// ------------------------------------------------------------
// HR MOVING AVERAGE
// Buffer melingkar untuk merata-ratakan 5 nilai BPM terakhir
// agar tampilan lebih stabil.
// ------------------------------------------------------------

#define HR_FILTER_SIZE 5

// Riwayat nilai BPM
float hrHistory[HR_FILTER_SIZE] = {
  0, 0, 0, 0, 0
};

// Posisi penulisan berikutnya pada buffer melingkar
int hrHistoryIndex = 0;

// Jumlah data valid yang sudah tersimpan (maksimal HR_FILTER_SIZE)
int hrHistoryCount = 0;


// ============================================================
// SP02 BUFFER
// Buffer sampel Red dan IR untuk algoritma SpO2.
// ============================================================

// Algoritma Maxim membutuhkan 100 sampel (4 detik pada 25 Hz)
#define SPO2_BUFFER_SIZE 100

uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];

// Jumlah sampel yang sudah terisi saat pengisian awal buffer
int spo2BufferCount = 0;

// Jumlah sampel baru sejak perhitungan SpO2 terakhir
int spo2NewSamples = 0;

// Penanda buffer awal sudah penuh (100 sampel)
bool spo2BufferReady = false;


// ============================================================
// SP02 RAW RESULT
// Hasil mentah dari algoritma SpO2.
// ============================================================

// Nilai SpO2 mentah (%)
int32_t spo2Raw = 0;

// Penanda apakah hasil SpO2 terakhir valid (1 = valid)
int8_t validSpO2 = 0;


// ============================================================
// SP02 FILTER
// Moving average untuk menghaluskan nilai SpO2.
// ============================================================

#define SPO2_FILTER_SIZE 5

// Riwayat nilai SpO2
float spo2History[SPO2_FILTER_SIZE] = {
  0, 0, 0, 0, 0
};

// Posisi penulisan berikutnya pada buffer melingkar
int spo2HistoryIndex = 0;

// Jumlah data valid yang sudah tersimpan
int spo2HistoryCount = 0;

// SpO2 hasil rata-rata, ini yang ditampilkan dan dikirim
float spo2Filtered = 0;


// ============================================================
// TEMPERATURE
// Variabel untuk pembacaan suhu non-blocking DS18B20.
// ============================================================

// Suhu terakhir yang berhasil dibaca (derajat Celsius)
float suhuAktual = 0;

// Penanda apakah pembacaan suhu terakhir valid
bool suhuValid = false;

// Waktu (millis) saat siklus pembacaan suhu terakhir dimulai
unsigned long waktuSuhuTerakhir = 0;

// Waktu (millis) saat permintaan konversi suhu dikirim
unsigned long waktuRequestSuhu = 0;

// Penanda bahwa konversi suhu sedang berlangsung
bool suhuSedangDiproses = false;

// Jarak antar siklus pembacaan suhu (ms)
const unsigned long SUHU_INTERVAL = 1000;

// Waktu tunggu konversi sensor (ms), cukup untuk resolusi 9 bit
const unsigned long SUHU_CONVERSION_TIME = 100;


// ============================================================
// STATUS
// Status sistem yang ditampilkan di OLED dan dikirim via MQTT.
// Nilai yang mungkin: "MENUNGGU SENSOR", "MENDETEKSI",
// "NORMAL", "ANOMALI".
// ============================================================

String statusSistem = "MENUNGGU SENSOR";


// ============================================================
// ANOMALY THRESHOLD
// Batas nilai untuk menentukan kondisi anomali.
// ============================================================

// Detak jantung normal: 50 - 100 BPM
const float HR_LOW = 50.0;
const float HR_HIGH = 100.0;

// SpO2 di bawah 90% dianggap rendah
const float SPO2_LOW = 90.0;

// Suhu di atas 38 derajat Celsius dianggap demam
const float TEMP_HIGH = 38.0;


// ============================================================
// TIMER
// Penyimpan waktu (millis) untuk tugas-tugas periodik
// agar program tidak memakai delay().
// ============================================================

// Update layar OLED terakhir
unsigned long lastOLEDUpdate = 0;

// Publish MQTT terakhir
unsigned long lastMQTTPublish = 0;

// Percobaan koneksi WiFi terakhir
unsigned long lastWiFiAttempt = 0;

// Percobaan koneksi MQTT terakhir
unsigned long lastMQTTAttempt = 0;

// Waktu buzzer terakhir dinyalakan
unsigned long lastBuzzerTime = 0;

// Waktu terjadwal untuk mematikan buzzer (0 = tidak terjadwal)
unsigned long buzzerOffTime = 0;


// ============================================================
// CONNECTION STATE
// Penanda proses koneksi yang sedang berjalan.
// ============================================================

// True saat WiFi sedang mencoba terhubung
bool wifiConnecting = false;

// True saat MQTT sedang mencoba terhubung
bool mqttConnecting = false;


// ============================================================
// RESET VITALS
// Mengosongkan seluruh data HR dan SpO2. Dipanggil saat jari
// dilepas supaya pengukuran berikutnya dimulai dari awal.
// ============================================================

void resetVitals() {

  // Nol-kan nilai HR
  bpmRaw = 0;

  bpmFiltered = 0;

  // Nol-kan nilai SpO2
  spo2Raw = 0;

  spo2Filtered = 0;

  validSpO2 = 0;


  // HR history
  // Kosongkan buffer rata-rata HR
  for (int i = 0; i < HR_FILTER_SIZE; i++) {
    hrHistory[i] = 0;
  }

  hrHistoryIndex = 0;
  hrHistoryCount = 0;


  // SpO2 history
  // Kosongkan buffer rata-rata SpO2
  for (int i = 0; i < SPO2_FILTER_SIZE; i++) {
    spo2History[i] = 0;
  }

  spo2HistoryIndex = 0;
  spo2HistoryCount = 0;


  // SpO2 buffer
  // Ulangi pengisian buffer sampel dari awal
  spo2BufferCount = 0;
  spo2NewSamples = 0;
  spo2BufferReady = false;


  // Beat
  // Lupakan referensi detak sebelumnya
  firstBeatDetected = false;
  lastBeatSample = 0;
}


// ============================================================
// ADD HR HISTORY
// Menambahkan nilai BPM baru ke buffer melingkar, lalu
// menghitung rata-ratanya (moving average) ke bpmFiltered.
// Nilai di luar 40 - 180 BPM dibuang.
// ============================================================

void addHR(float value) {

  // Abaikan nilai yang tidak masuk akal secara fisiologis
  if (value < 40 || value > 180) {
    return;
  }


  // Simpan nilai pada posisi saat ini
  hrHistory[hrHistoryIndex] = value;

  // Geser indeks ke posisi berikutnya
  hrHistoryIndex++;

  // Jika melewati ujung buffer, kembali ke awal (melingkar)
  if (hrHistoryIndex >= HR_FILTER_SIZE) {
    hrHistoryIndex = 0;
  }

  // Tambah jumlah data valid sampai penuh
  if (hrHistoryCount < HR_FILTER_SIZE) {
    hrHistoryCount++;
  }


  // Jumlahkan seluruh data valid pada buffer
  float total = 0;

  for (int i = 0; i < hrHistoryCount; i++) {
    total += hrHistory[i];
  }

  // Rata-rata = total / jumlah data
  bpmFiltered =
    total / hrHistoryCount;
}


// ============================================================
// ADD SPO2 HISTORY
// Menambahkan nilai SpO2 baru ke buffer melingkar, lalu
// menghitung rata-ratanya ke spo2Filtered.
// Nilai di luar 70 - 100 % dibuang.
// ============================================================

void addSpO2(float value) {

  // Abaikan nilai yang tidak masuk akal
  if (value < 70 || value > 100) {
    return;
  }


  // Simpan nilai pada posisi saat ini
  spo2History[spo2HistoryIndex] = value;

  // Geser indeks ke posisi berikutnya
  spo2HistoryIndex++;

  // Kembali ke awal jika melewati ujung buffer (melingkar)
  if (spo2HistoryIndex >= SPO2_FILTER_SIZE) {
    spo2HistoryIndex = 0;
  }

  // Tambah jumlah data valid sampai penuh
  if (spo2HistoryCount < SPO2_FILTER_SIZE) {
    spo2HistoryCount++;
  }


  // ----------------------------------------------------------
  // Average
  // Hitung rata-rata dari seluruh data valid.
  // ----------------------------------------------------------

  float total = 0;

  for (int i = 0; i < spo2HistoryCount; i++) {
    total += spo2History[i];
  }

  spo2Filtered =
    total / spo2HistoryCount;
}


// ============================================================
// PROCESS HEART RATE
// Mendeteksi detak dari sinyal IR, menghitung selang waktu
// antar detak, lalu mengubahnya menjadi BPM.
// ============================================================
//
// IMPORTANT:
//
// Kita memakai SAMPLE COUNT berdasarkan FIFO = 25 Hz.
// Jangan gunakan 100 Hz di sini karena sampleAverage = 4.
//

void processHeartRate(uint32_t irValue) {

  // Cek apakah sampel IR ini merupakan puncak detak jantung
  bool beat = checkForBeat(irValue);


  // Tidak ada detak, tidak ada yang perlu dihitung
  if (!beat) {
    return;
  }


  // Nomor sampel saat detak ini terdeteksi
  unsigned long currentSample =
    totalSamples;


  Serial.print("[BEAT] Sample=");
  Serial.print(currentSample);


  // ----------------------------------------------------------
  // FIRST BEAT
  // Detak pertama hanya disimpan sebagai titik referensi,
  // belum bisa menghitung BPM karena butuh dua detak.
  // ----------------------------------------------------------

  if (!firstBeatDetected) {

    firstBeatDetected = true;

    lastBeatSample = currentSample;

    Serial.println(" | FIRST BEAT");

    return;
  }


  // ----------------------------------------------------------
  // SECOND / NEXT BEAT
  // Hitung jumlah sampel antara detak ini dan detak sebelumnya.
  // ----------------------------------------------------------

  unsigned long deltaSamples =
    currentSample - lastBeatSample;


  Serial.print(" | DeltaSamples=");
  Serial.print(deltaSamples);


  // ----------------------------------------------------------
  // Valid beat interval
  //
  // 25 Hz:
  //
  // 8 samples  = 320 ms = 187.5 BPM
  // 38 samples = 1.52 s = 39.5 BPM
  // ----------------------------------------------------------

  // Interval terlalu pendek: kemungkinan noise, abaikan
  if (deltaSamples < 8) {

    Serial.println(" | REJECTED: TOO FAST");

    return;
  }


  // Interval terlalu panjang: kemungkinan ada detak terlewat
  if (deltaSamples > 38) {

    Serial.println(" | REJECTED: TOO SLOW");

    // tetap gunakan beat sekarang
    // sebagai referensi berikutnya
    lastBeatSample = currentSample;

    return;
  }


  // ----------------------------------------------------------
  // HITUNG BPM
  // BPM = (60 detik x laju sampel) / jumlah sampel per detak
  // ----------------------------------------------------------

  float newBPM =
    (60.0 * FIFO_SAMPLE_RATE) /
    deltaSamples;


  Serial.print(" | BPM_RAW=");
  Serial.print(newBPM, 1);


  // Terima hanya jika BPM berada dalam rentang wajar
  if (newBPM >= 40 &&
      newBPM <= 180) {

    // Simpan BPM mentah
    bpmRaw = newBPM;

    // Masukkan ke moving average
    addHR(newBPM);

    // Jadikan detak ini referensi untuk detak berikutnya
    lastBeatSample = currentSample;


    Serial.print(
      " | BPM_FILTERED="
    );

    Serial.println(
      bpmFiltered,
      1
    );

  } else {

    Serial.println(
      " | BPM REJECTED"
    );
  }
}


// ============================================================
// PROCESS SP02 BUFFER
// Mengumpulkan sampel Red dan IR ke dalam buffer. Setelah
// buffer penuh, SpO2 dihitung ulang setiap 25 sampel baru
// (sliding window).
// ============================================================

void addSpO2Sample(
  uint32_t redValue,
  uint32_t irValue
) {

  // ==========================================================
  // FILL INITIAL BUFFER
  // Tahap 1: isi buffer 100 sampel pertama.
  // ==========================================================

  if (!spo2BufferReady) {

    // Simpan sampel pada posisi berikutnya
    redBuffer[spo2BufferCount] =
      redValue;

    irBuffer[spo2BufferCount] =
      irValue;

    spo2BufferCount++;


    // --------------------------------------------------------
    // Buffer penuh
    // Hitung SpO2 untuk pertama kali.
    // --------------------------------------------------------

    if (spo2BufferCount >= SPO2_BUFFER_SIZE) {

      spo2BufferReady = true;

      spo2NewSamples = 0;


      calculateSpO2();

    }

    return;
  }


  // ==========================================================
  // BUFFER SUDAH PENUH
  //
  // Setiap 25 sample:
  // - buang 25 sample lama
  // - simpan 25 sample baru
  // ==========================================================

  // Pada sampel baru pertama dalam satu siklus,
  // geser data lama ke depan untuk memberi ruang
  if (spo2NewSamples == 0) {

    // --------------------------------------------------------
    // Shift 75 sample terakhir
    // Data indeks 25-99 dipindah ke indeks 0-74.
    // --------------------------------------------------------

    for (
      int i = 25;
      i < SPO2_BUFFER_SIZE;
      i++
    ) {

      redBuffer[i - 25] =
        redBuffer[i];

      irBuffer[i - 25] =
        irBuffer[i];
    }
  }


  // ----------------------------------------------------------
  // Masukkan sample baru
  // Sampel baru ditulis mulai indeks 75 (slot kosong di akhir).
  // ----------------------------------------------------------

  redBuffer[75 + spo2NewSamples] =
    redValue;

  irBuffer[75 + spo2NewSamples] =
    irValue;


  spo2NewSamples++;


  // ----------------------------------------------------------
  // 25 sample baru lengkap
  // Hitung ulang SpO2 dan mulai siklus berikutnya.
  // ----------------------------------------------------------

  if (spo2NewSamples >= 25) {

    spo2NewSamples = 0;

    calculateSpO2();
  }
}


// ============================================================
// CALCULATE SPO2
// Memanggil algoritma Maxim untuk menghitung SpO2 dari isi
// buffer, lalu memvalidasi dan memfilter hasilnya.
// ============================================================

void calculateSpO2() {

  // Variabel penampung hasil dari algoritma Maxim
  int32_t tempSpO2;

  int8_t tempValidSpO2;

  int32_t tempHeartRate;

  int8_t tempValidHeartRate;


  // Jalankan algoritma SpO2 dan HR bawaan Maxim
  maxim_heart_rate_and_oxygen_saturation(

    irBuffer,

    SPO2_BUFFER_SIZE,

    redBuffer,

    &tempSpO2,

    &tempValidSpO2,

    &tempHeartRate,

    &tempValidHeartRate
  );


  // ----------------------------------------------------------
  // SpO2
  //
  // Untuk HR kita sengaja TIDAK menggunakan tempHeartRate.
  // HR tetap berasal dari checkForBeat() yang sudah terbukti
  // bekerja pada sensor kamu.
  // ----------------------------------------------------------

  // Terima hasil hanya jika algoritma menyatakan valid
  // dan nilainya berada dalam rentang 70 - 100 %
  if (
    tempValidSpO2 &&
    tempSpO2 >= 70 &&
    tempSpO2 <= 100
  ) {

    // Simpan nilai mentah dan tandai valid
    spo2Raw =
      tempSpO2;

    validSpO2 = 1;

    // Masukkan ke moving average
    addSpO2(
      (float)tempSpO2
    );


    Serial.print(
      "[SpO2] RAW="
    );

    Serial.print(
      tempSpO2
    );

    Serial.print(
      " | FILTERED="
    );

    Serial.println(
      spo2Filtered,
      1
    );

  } else {

    // Hasil tidak valid, tandai dan jangan ubah nilai filter
    validSpO2 = 0;

    Serial.println(
      "[SpO2] INVALID"
    );
  }
}


// ============================================================
// PROCESS MAX30102 FIFO
// Membaca semua sampel yang tersedia di FIFO sensor, lalu
// meneruskannya ke proses deteksi jari, HR, dan SpO2.
// ============================================================

void processMAX30102() {

  // Ambil data terbaru dari sensor ke dalam FIFO library
  particleSensor.check();


  // Proses seluruh sampel yang tersedia
  while (
    particleSensor.available()
  ) {

    // Baca nilai Red dan IR dari sampel saat ini
    uint32_t redValue =
      particleSensor.getFIFORed();

    uint32_t irValue =
      particleSensor.getFIFOIR();


    // --------------------------------------------------------
    // Sample counter
    // Hitung total sampel sebagai basis waktu perhitungan HR.
    // --------------------------------------------------------

    totalSamples++;


    // --------------------------------------------------------
    // FINGER
    // Jika IR melebihi threshold, jari menempel di sensor.
    // --------------------------------------------------------

    if (
      irValue >
      FINGER_THRESHOLD
    ) {

      fingerPresent = true;

      lastFingerDetected =
        millis();

    }


    // --------------------------------------------------------
    // HR
    // Hitung detak jantung hanya jika jari menempel.
    // --------------------------------------------------------

    if (irValue > FINGER_THRESHOLD) {

      processHeartRate(
        irValue
      );
    }


    // --------------------------------------------------------
    // SpO2
    // Kumpulkan sampel SpO2 hanya jika jari menempel.
    // --------------------------------------------------------

    if (irValue > FINGER_THRESHOLD) {

      addSpO2Sample(
        redValue,
        irValue
      );
    }


    // --------------------------------------------------------
    // Next FIFO sample
    // Lanjut ke sampel berikutnya di FIFO.
    // --------------------------------------------------------

    particleSensor.nextSample();
  }
}


// ============================================================
// CHECK FINGER
// Jika jari tidak terdeteksi lebih dari FINGER_TIMEOUT,
// anggap jari dilepas dan reset seluruh data pengukuran.
// ============================================================

void checkFingerTimeout() {

  // Tidak ada jari, tidak perlu dicek
  if (!fingerPresent) {
    return;
  }


  // Sudah terlalu lama sejak terakhir jari terdeteksi
  if (
    millis() -
    lastFingerDetected >
    FINGER_TIMEOUT
  ) {

    fingerPresent = false;

    // Bersihkan data agar pengukuran baru dimulai dari nol
    resetVitals();

    statusSistem =
      "MENUNGGU SENSOR";
  }
}


// ============================================================
// TEMPERATURE
// Pembacaan suhu DS18B20 secara non-blocking dalam dua tahap:
// 1. Kirim perintah konversi suhu.
// 2. Setelah SUHU_CONVERSION_TIME, baca hasilnya.
// Dengan cara ini loop() tidak tertahan oleh delay.
// ============================================================

void updateTemperature() {

  unsigned long now =
    millis();


  // ==========================================================
  // START REQUEST
  // Tahap 1: minta sensor memulai konversi suhu setiap
  // SUHU_INTERVAL jika tidak ada konversi yang berjalan.
  // ==========================================================

  if (
    !suhuSedangDiproses &&
    now - waktuSuhuTerakhir >= SUHU_INTERVAL
  ) {

    sensors.requestTemperatures();

    waktuRequestSuhu = now;

    waktuSuhuTerakhir = now;

    suhuSedangDiproses = true;
  }


  // ==========================================================
  // READ AFTER CONVERSION
  // Tahap 2: setelah waktu konversi cukup, ambil hasilnya.
  // ==========================================================

  if (
    suhuSedangDiproses &&
    now - waktuRequestSuhu >=
    SUHU_CONVERSION_TIME
  ) {

    // Baca suhu dari sensor pertama pada bus OneWire
    float temp =
      sensors.getTempCByIndex(0);


    // Valid jika sensor terhubung dan nilai masuk akal
    if (
      temp != DEVICE_DISCONNECTED_C &&
      temp > -20 &&
      temp < 70
    ) {

      suhuAktual = temp;

      suhuValid = true;

    } else {

      // Sensor terlepas atau pembacaan tidak masuk akal
      suhuValid = false;
    }


    // Konversi selesai, siap untuk siklus berikutnya
    suhuSedangDiproses = false;
  }
}


// ============================================================
// WIFI SERVICE
// Mengelola koneksi WiFi secara non-blocking: mencoba
// terhubung, menangani timeout, dan mencoba ulang.
// ============================================================

void serviceWiFi() {

  // Ambil status koneksi WiFi saat ini
  wl_status_t state =
    WiFi.status();


  // ==========================================================
  // CONNECTED
  // Sudah terhubung, tidak ada yang perlu dilakukan.
  // ==========================================================

  if (state == WL_CONNECTED) {

    wifiConnecting = false;

    return;
  }


  // ==========================================================
  // JIKA SEDANG CONNECTING
  // Tunggu hingga 15 detik; jika lewat, batalkan dan ulangi.
  // ==========================================================

  if (wifiConnecting) {

    if (
      millis() -
      lastWiFiAttempt >
      15000
    ) {

      Serial.println(
        "WiFi timeout, retry..."
      );

      WiFi.disconnect();

      wifiConnecting = false;

    } else {

      // Masih menunggu, keluar dulu
      return;
    }
  }


  // ==========================================================
  // RETRY
  // Beri jeda minimal 5 detik antar percobaan koneksi.
  // ==========================================================

  if (
    millis() -
    lastWiFiAttempt <
    5000
  ) {

    return;
  }


  Serial.print(
    "Connecting WiFi: "
  );

  Serial.println(
    WIFI_SSID
  );


  // Mulai koneksi sebagai station (client WiFi)
  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASS
  );


  // Catat waktu percobaan dan tandai sedang connecting
  lastWiFiAttempt =
    millis();

  wifiConnecting = true;
}


// ============================================================
// MQTT CONNECT
// Mengelola koneksi ke broker MQTT. Hanya berjalan jika WiFi
// sudah terhubung; mencoba ulang tiap 5 detik jika terputus.
// ============================================================

void serviceMQTT() {

  // MQTT membutuhkan WiFi, jadi keluar jika WiFi belum siap
  if (
    WiFi.status() != WL_CONNECTED
  ) {

    return;
  }


  // Jika sudah terhubung, jalankan loop MQTT (keep-alive)
  if (
    mqttClient.connected()
  ) {

    mqttClient.loop();

    mqttConnecting = false;

    return;
  }


  // Jika sedang proses connect, tunggu
  if (mqttConnecting) {

    return;
  }


  // Beri jeda minimal 5 detik antar percobaan koneksi
  if (
    millis() -
    lastMQTTAttempt <
    5000
  ) {

    return;
  }


  Serial.println(
    "Connecting MQTT..."
  );


  mqttConnecting = true;

  lastMQTTAttempt =
    millis();


  // Buat client ID unik dari MAC address ESP32
  String clientId =
    "PhysioMonitor-" +
    String(
      (uint32_t)
      ESP.getEfuseMac(),
      HEX
    );


  // Coba terhubung ke broker
  if (
    mqttClient.connect(
      clientId.c_str()
    )
  ) {

    Serial.println(
      "MQTT connected"
    );

    mqttConnecting = false;

  } else {

    // Tampilkan kode error MQTT untuk debugging
    Serial.print(
      "MQTT failed, state="
    );

    Serial.println(
      mqttClient.state()
    );

    mqttConnecting = false;
  }
}


// ============================================================
// STATUS
// Menentukan status sistem berdasarkan kondisi jari dan
// nilai HR, SpO2, serta suhu.
// ============================================================

void updateStatus() {

  // Jika tidak ada jari, sistem menunggu sensor
  if (!fingerPresent) {

    statusSistem =
      "MENUNGGU SENSOR";

    return;
  }


  // ----------------------------------------------------------
  // Belum punya HR + SpO2
  // Jari sudah menempel tetapi data belum lengkap.
  // ----------------------------------------------------------

  if (
    bpmFiltered <= 0 ||
    spo2Filtered <= 0
  ) {

    statusSistem =
      "MENDETEKSI";

    return;
  }


  // ----------------------------------------------------------
  // ANOMALY
  // Periksa apakah ada nilai yang keluar dari batas normal.
  // ----------------------------------------------------------

  bool anomaly = false;


  // HR terlalu rendah atau terlalu tinggi
  if (
    bpmFiltered < HR_LOW ||
    bpmFiltered > HR_HIGH
  ) {

    anomaly = true;
  }


  // SpO2 terlalu rendah
  if (
    spo2Filtered < SPO2_LOW
  ) {

    anomaly = true;
  }


  // Suhu terlalu tinggi (hanya dicek jika pembacaan valid)
  if (
    suhuValid &&
    suhuAktual > TEMP_HIGH
  ) {

    anomaly = true;
  }


  // Tetapkan status akhir
  if (anomaly) {

    statusSistem =
      "ANOMALI";

  } else {

    statusSistem =
      "NORMAL";
  }
}


// ============================================================
// BUZZER
// Membunyikan buzzer secara berkala (beep 150 ms setiap
// 1000 ms) selama status ANOMALI, tanpa memakai delay().
// ============================================================

void updateBuzzer() {

  unsigned long now =
    millis();


  // ----------------------------------------------------------
  // Jika bukan anomaly
  // Pastikan buzzer mati dan hapus jadwal matinya.
  // ----------------------------------------------------------

  if (
    statusSistem != "ANOMALI"
  ) {

    digitalWrite(
      BUZZER_PIN,
      BUZZER_MATI
    );

    buzzerOffTime = 0;

    return;
  }


  // ----------------------------------------------------------
  // Matikan buzzer setelah 150ms
  // Jika waktu mati terjadwal sudah tercapai, matikan buzzer.
  // ----------------------------------------------------------

  if (
    buzzerOffTime != 0 &&
    now >= buzzerOffTime
  ) {

    digitalWrite(
      BUZZER_PIN,
      BUZZER_MATI
    );

    buzzerOffTime = 0;
  }


  // ----------------------------------------------------------
  // Beep setiap 1000ms
  // Nyalakan buzzer dan jadwalkan mati 150 ms kemudian.
  // ----------------------------------------------------------

  if (
    buzzerOffTime == 0 &&
    now - lastBuzzerTime >= 1000
  ) {

    lastBuzzerTime = now;

    digitalWrite(
      BUZZER_PIN,
      BUZZER_NYALA
    );

    buzzerOffTime =
      now + 150;
  }
}


// ============================================================
// OLED
// Memperbarui tampilan layar OLED setiap 200 ms:
// judul, HR, SpO2, suhu, dan status sistem.
// ============================================================

void updateOLED() {

  // Batasi laju refresh layar menjadi setiap 200 ms
  if (
    millis() -
    lastOLEDUpdate <
    200
  ) {

    return;
  }


  lastOLEDUpdate =
    millis();


  // Hapus isi layar sebelum menggambar ulang
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );


  // ==========================================================
  // TITLE
  // Baris judul di bagian atas layar.
  // ==========================================================

  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println(
    "PHYSIO MONITOR"
  );


  // ==========================================================
  // HR
  // Tampilkan BPM, atau "--" jika belum tersedia.
  // ==========================================================

  display.setCursor(0, 12);

  display.print("HR: ");

  if (bpmFiltered > 0) {

    display.print(
      bpmFiltered,
      1
    );

    display.println(
      " BPM"
    );

  } else {

    display.println(
      "--"
    );
  }


  // ==========================================================
  // SPO2
  // Tampilkan SpO2, atau "--" jika belum tersedia.
  // ==========================================================

  display.setCursor(0, 25);

  display.print(
    "SpO2: "
  );

  if (spo2Filtered > 0) {

    display.print(
      spo2Filtered,
      1
    );

    display.println(
      " %"
    );

  } else {

    display.println(
      "--"
    );
  }


  // ==========================================================
  // TEMPERATURE
  // Tampilkan suhu, atau "--" jika pembacaan tidak valid.
  // ==========================================================

  display.setCursor(0, 38);

  display.print(
    "Temp: "
  );

  if (suhuValid) {

    display.print(
      suhuAktual,
      1
    );

    display.println(
      " C"
    );

  } else {

    display.println(
      "--"
    );
  }


  // ==========================================================
  // STATUS
  // Tampilkan status sistem di baris paling bawah.
  // ==========================================================

  display.setCursor(0, 51);

  display.print(
    statusSistem
  );


  // Kirim buffer gambar ke layar agar tampil
  display.display();
}


// ============================================================
// MQTT PUBLISH
// Mengirim data HR, SpO2, suhu, dan status dalam format JSON
// ke broker MQTT setiap 1 detik.
// ============================================================

void publishMQTT() {

  // Jangan kirim jika belum terhubung ke broker
  if (
    !mqttClient.connected()
  ) {

    return;
  }


  // Batasi pengiriman menjadi satu kali per 1000 ms
  if (
    millis() -
    lastMQTTPublish <
    1000
  ) {

    return;
  }


  lastMQTTPublish =
    millis();


  // ==========================================================
  // JSON
  // Susun payload secara manual, contoh hasil:
  // {"hr":75.0,"spo2":98.0,"temp":36.5,"status":"NORMAL"}
  // Nilai yang belum tersedia dikirim sebagai 0.0.
  // ==========================================================

  String payload = "{";


  // HR
  payload += "\"hr\":";

  if (bpmFiltered > 0) {

    payload +=
      String(
        bpmFiltered,
        1
      );

  } else {

    payload += "0.0";
  }


  // SpO2
  payload += ",\"spo2\":";

  if (spo2Filtered > 0) {

    payload +=
      String(
        spo2Filtered,
        1
      );

  } else {

    payload += "0.0";
  }


  // Temperature
  payload += ",\"temp\":";

  if (suhuValid) {

    payload +=
      String(
        suhuAktual,
        1
      );

  } else {

    payload += "0.0";
  }


  // Status
  payload +=
    ",\"status\":\"";

  payload +=
    statusSistem;

  payload +=
    "\"}";


  // Kirim payload ke topik MQTT
  mqttClient.publish(
    MQTT_TOPIC,
    payload.c_str()
  );


  // Tampilkan payload di Serial Monitor untuk pengecekan
  Serial.print(
    "MQTT -> "
  );

  Serial.println(
    payload
  );
}


// ============================================================
// DEBUG SERIAL
// Mencetak ringkasan seluruh kondisi sistem ke Serial Monitor
// setiap 1 detik: IR, RED, HR, SpO2, suhu, status, jumlah
// sampel, serta status WiFi dan MQTT.
// ============================================================

void printDebug(uint32_t irValue,
                uint32_t redValue) {

  // Waktu debug terakhir
  static unsigned long lastDebug = 0;


  // Batasi pencetakan menjadi satu kali per detik
  if (
    millis() -
    lastDebug <
    1000
  ) {

    return;
  }


  lastDebug =
    millis();


  // Nilai mentah sensor
  Serial.print(
    "IR: "
  );

  Serial.print(
    irValue
  );

  Serial.print(
    " | RED: "
  );

  Serial.print(
    redValue
  );

  // Detak jantung
  Serial.print(
    " | BPM: "
  );

  if (bpmFiltered > 0) {

    Serial.print(
      bpmFiltered,
      1
    );

  } else {

    Serial.print("--");
  }


  // Saturasi oksigen
  Serial.print(
    " | SpO2: "
  );

  if (spo2Filtered > 0) {

    Serial.print(
      spo2Filtered,
      1
    );

  } else {

    Serial.print("--");
  }


  // Suhu
  Serial.print(
    " | Temp: "
  );

  if (suhuValid) {

    Serial.print(
      suhuAktual,
      1
    );

  } else {

    Serial.print("--");
  }


  // Status sistem
  Serial.print(
    " | Status: "
  );

  Serial.print(
    statusSistem
  );


  // Total sampel yang sudah dibaca
  Serial.print(
    " | Samples: "
  );

  Serial.print(
    totalSamples
  );


  // Status koneksi WiFi
  Serial.print(
    " | WiFi: "
  );

  Serial.print(
    WiFi.status() ==
    WL_CONNECTED
      ? "OK"
      : "OFF"
  );


  // Status koneksi MQTT
  Serial.print(
    " | MQTT: "
  );

  Serial.println(
    mqttClient.connected()
      ? "OK"
      : "OFF"
  );
}


// ============================================================
// SETUP
// Dijalankan sekali saat ESP32 menyala: inisialisasi serial,
// buzzer, sensor, OLED, MQTT, dan WiFi.
// ============================================================

void setup() {

  // Mulai komunikasi serial untuk debugging
  Serial.begin(115200);

  delay(1000);


  // Banner awal di Serial Monitor
  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    " PHYSIO MONITOR"
  );

  Serial.println(
    " ESP32 + MAX30102 + DS18B20"
  );

  Serial.println(
    " HR + SpO2 + MQTT"
  );

  Serial.println(
    "========================================"
  );


  // ==========================================================
  // BUZZER
  // Atur pin buzzer sebagai output dan pastikan awalnya mati.
  // ==========================================================

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    BUZZER_PIN,
    BUZZER_MATI
  );


  // ==========================================================
  // MAX30102 I2C
  // Mulai bus I2C utama dengan kecepatan 400 kHz (fast mode).
  // ==========================================================

  Wire.begin(
    MAX_SDA,
    MAX_SCL
  );

  Wire.setClock(400000);


  // Jika sensor tidak ditemukan, hentikan program
  if (
    !particleSensor.begin(
      Wire,
      I2C_SPEED_FAST
    )
  ) {

    Serial.println(
      "MAX30102 tidak ditemukan!"
    );

    while (1) {

      delay(1000);
    }
  }


  Serial.println(
    "MAX30102 OK"
  );


  // ==========================================================
  // MAX30102 CONFIG
  // Terapkan konfigurasi sensor yang sudah didefinisikan.
  // ==========================================================

  particleSensor.setup(
    LED_BRIGHTNESS,
    SAMPLE_AVERAGE,
    LED_MODE,
    SAMPLE_RATE,
    PULSE_WIDTH,
    ADC_RANGE
  );


  // Atur intensitas LED Red dan IR
  particleSensor.setPulseAmplitudeRed(
    LED_BRIGHTNESS
  );

  particleSensor.setPulseAmplitudeIR(
    LED_BRIGHTNESS
  );

  // LED hijau tidak dipakai, matikan
  particleSensor.setPulseAmplitudeGreen(
    0
  );


  // ==========================================================
  // DS18B20
  // Mulai sensor suhu dengan resolusi 9 bit (konversi cepat)
  // dan nonaktifkan mode tunggu agar tidak memblokir program.
  // ==========================================================

  sensors.begin();

  sensors.setResolution(9);

  sensors.setWaitForConversion(false);


  // ==========================================================
  // OLED
  // Mulai bus I2C kedua khusus OLED pada 400 kHz.
  // ==========================================================

  oledWire.begin(
    OLED_SDA,
    OLED_SCL,
    400000
  );


  // Inisialisasi layar OLED pada alamat I2C 0x3C
  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      0x3C
    )
  ) {

    // OLED gagal, program tetap berjalan tanpa layar
    Serial.println(
      "OLED tidak ditemukan."
    );

  } else {

    // Tampilkan layar pembuka
    display.clearDisplay();

    display.setTextColor(
      SSD1306_WHITE
    );

    display.setTextSize(1);

    display.setCursor(0, 0);

    display.println(
      "PHYSIO MONITOR"
    );

    display.println();

    display.println(
      "Initializing..."
    );

    display.display();
  }


  // ==========================================================
  // MQTT
  // Atur alamat broker dan port MQTT.
  // ==========================================================

  mqttClient.setServer(
    MQTT_BROKER,
    MQTT_PORT
  );


  // ==========================================================
  // WIFI
  // Siapkan mode station, auto-reconnect, dan nonaktifkan
  // penyimpanan kredensial ke flash.
  // ==========================================================

  WiFi.mode(WIFI_STA);

  WiFi.setAutoReconnect(true);

  WiFi.persistent(false);


  // ==========================================================
  // INITIAL TEMPERATURE
  // Atur agar pembacaan suhu pertama langsung dimulai pada
  // loop pertama, tanpa menunggu SUHU_INTERVAL.
  // ==========================================================

  waktuSuhuTerakhir =
    millis() -
    SUHU_INTERVAL;


  Serial.println();

  Serial.println(
    "System ready."
  );

  Serial.println(
    "Tempelkan jari pada MAX30102."
  );

  Serial.println();
}


// ============================================================
// LOOP
// Dijalankan berulang terus-menerus. Seluruh tugas dibuat
// non-blocking sehingga sensor, jaringan, layar, dan buzzer
// dapat berjalan bersamaan.
// ============================================================

void loop() {

  // ==========================================================
  // 1. SENSOR
  // Baca dan proses data MAX30102 (HR dan SpO2).
  // ==========================================================

  processMAX30102();


  // ==========================================================
  // 2. FINGER TIMEOUT
  // Cek apakah jari sudah dilepas.
  // ==========================================================

  checkFingerTimeout();


  // ==========================================================
  // 3. TEMPERATURE
  // Perbarui pembacaan suhu DS18B20.
  // ==========================================================

  updateTemperature();


  // ==========================================================
  // 4. STATUS
  // Tentukan status sistem (NORMAL / ANOMALI / dll).
  // ==========================================================

  updateStatus();


  // ==========================================================
  // 5. WIFI
  // Pertahankan koneksi WiFi.
  // ==========================================================

  serviceWiFi();


  // ==========================================================
  // 6. MQTT
  // Pertahankan koneksi MQTT.
  // ==========================================================

  serviceMQTT();


  // ==========================================================
  // 7. MQTT PUBLISH
  // Kirim data ke broker setiap 1 detik.
  // ==========================================================

  publishMQTT();


  // ==========================================================
  // 8. OLED
  // Perbarui tampilan layar.
  // ==========================================================

  updateOLED();


  // ==========================================================
  // 9. BUZZER
  // Kendalikan bunyi buzzer sesuai status.
  // ==========================================================

  updateBuzzer();


  // ==========================================================
  // DEBUG
  // Simpan nilai IR dan RED terakhir untuk dicetak ke Serial.
  // ==========================================================

  static uint32_t lastIR = 0;
  static uint32_t lastRED = 0;

  if (particleSensor.available()) {

    lastIR =
      particleSensor.getFIFOIR();

    lastRED =
      particleSensor.getFIFORed();
  }

  printDebug(
    lastIR,
    lastRED
  );
}
