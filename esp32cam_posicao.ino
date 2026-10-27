/*
 * ESP32-CAM (AI Thinker) - Detecção de 4 posições (0, 90, 180, 270 graus)
 *
 * IDEIA: a câmera fica de cima (22,5 cm de distância), olhando para um objeto
 * que gira em torno de um ponto (CX, CY). A marca (adesivo/fita) fica a 8 cm
 * do centro de rotação. O código acha onde está a marca, calcula o ângulo dela
 * em relação ao centro e arredonda para o múltiplo de 90 graus mais próximo.
 *
 * GEOMETRIA (estimada, calibre com /foto):
 *   OV2640 a 22,5 cm -> campo de ~23 x 17 cm -> ~14 px/cm em QVGA (320x240)
 *   Raio de 8 cm -> ~112 px (a meia-altura da imagem é 120 px)
 *
 * INTEGRAÇÃO COM A ESP32 DO MOTOR (ESP-NOW):
 *   A cada leitura a câmera ENVIA o ângulo real da marca (0-360, horário,
 *   0 = fase NOVA) para a ESP32 do motor, que usa isso para corrigir o desvio.
 *   O ESP-NOW usa o MESMO canal Wi-Fi do roteador. O canal aparece no Serial
 *   ao iniciar: coloque esse número em CANAL_WIFI no sketch do motor.
 *   (Requer Arduino-ESP32 core 3.x)
 *
 * Rotas:
 *   http://IP/         -> PÁGINA que mostra, ao vivo, a FASE mais próxima do objeto
 *   http://IP/posicao  -> JSON com a fase mais próxima, ângulo, desvio e pixels
 *   http://IP/foto     -> foto em cinza COM AS ANGULAÇÕES DESENHADAS:
 *        - anel de busca (dois círculos), círculo fino de 8 cm e cruz no centro
 *        - linhas cheias nos ângulos 0, 90, 180, 270 com o número escrito
 *        - linhas pontilhadas em 45, 135, 225, 315 (limites entre posições)
 *        - uma linha do centro até a marca detectada + círculo sobre ela
 *        - canto superior esquerdo: ângulo medido, posição arredondada e
 *          o desvio em relação a ela (ex.: +3.4 = 3,4 graus passou do alvo)
 *
 * Fases: 0 = NOVA (marca para CIMA), 1 = CRESCENTE (DIREITA),
 *        2 = CHEIA (BAIXO), 3 = MINGUANTE (ESQUERDA)
 * (sentido horário, visto pela câmera)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// ================== CONFIGURE AQUI ==================
const char* WIFI_SSID  = "";
const char* WIFI_SENHA = "";

// Centro de rotação do objeto na imagem (QVGA = 320x240 -> centro é 160,120)
const int CX = 160;
const int CY = 120;

// Geometria física
const float PX_POR_CM = 14.0;     // ~14 px/cm a 22,5 cm (calibre se precisar)
const float RAIO_CM   = 8.0;      // raio da circunferência onde fica a marca
const int RAIO_MARCA  = (int)(RAIO_CM * PX_POR_CM + 0.5f);   // ~112 px

// Anel de busca ao redor de 8 cm (a imagem tem só 120 px de meia-altura)
const int RAIO_MIN = 80;          // ~5,7 cm
const int RAIO_MAX = 117;         // ~8,4 cm (limite da imagem)

// Como a marca é vista
const bool MARCA_ESCURA = true;   // true: marca escura em fundo claro | false: o contrário
const uint8_t LIMIAR    = 90;     // 0-255: separa "escuro" de "claro"
const int MIN_PIXELS    = 15;     // mínimo de pixels para aceitar que achou a marca
                                  // (a 14 px/cm, uma fita de 5 mm tem ~7 px de largura;
                                  //  reduza se a marca for pequena)

// Gira o "zero" se a posição 0 não estiver no sentido que você quer
const float AJUSTE_ANGULO = 0.4;  // em graus
// ====================================================

// Pinos - AI Thinker
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_FLASH_GPIO     4

// Fases da Lua, na ordem do ciclo (sentido horário visto pela câmera):
//   0 = Nova (0°) -> 1 = Crescente (90°) -> 2 = Cheia (180°) -> 3 = Minguante (270°)
const char* NOMES[4] = {"NOVA (0)", "CRESCENTE (90)", "CHEIA (180)", "MINGUANTE (270)"};

// ---------- Pacote enviado para a ESP32 do motor ----------
// (a mesma struct precisa existir, idêntica, no sketch do motor)
typedef struct __attribute__((packed)) {
  uint8_t  magic;    // 0xA5 - identifica que o pacote é nosso
  uint8_t  valido;   // 1 = marca encontrada e leitura estável
  uint16_t seq;      // contador de pacotes
  float    angulo;   // 0-360, horário, 0 = cima (fase NOVA)
  uint16_t pixels;   // quantidade de pixels da marca
} PacoteAngulo;

// Broadcast: não precisa saber o MAC da outra placa
uint8_t ENDERECO_BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Resultado mais recente (compartilhado com o servidor web)
volatile int   posicaoEstavel = -1;
volatile float ultimoAngulo   = 0;
volatile int   ultimosPixels  = 0;

// Fase (0-3) MAIS PRÓXIMA do ângulo medido, atualizada a cada leitura válida
volatile int   proximaFase    = -1;     // -1 = ainda não leu nada
volatile float desvioFase     = 0;      // graus até essa fase (-45 a +45; + = passou, - = falta)
volatile bool  detectadoAgora = false;  // a marca foi vista na leitura mais recente?

httpd_handle_t servidor = NULL;

// ---------- Câmera em ESCALA DE CINZA (1 byte por pixel, fácil de analisar) ----------
bool iniciarCamera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;   c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;   c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size   = FRAMESIZE_QVGA;          // 320x240
  c.fb_count     = psramFound() ? 2 : 1;
  c.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;      // sempre o quadro mais novo

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_exposure_ctrl(s, 1);  // exposição automática
  s->set_gain_ctrl(s, 1);
  return true;
}

// ---------- Detecção ----------
// Analisa um quadro JÁ CAPTURADO. Retorna true se achou a marca.
//   angulo   : 0-360, horário, 0 = cima (já com AJUSTE_ANGULO)
//   pixels   : quantos pixels da marca foram encontrados
//   mx, my   : posição da marca em pixels, relativa ao centro (x p/ direita, y p/ baixo)
bool analisarFrame(camera_fb_t* fb, float &angulo, int &pixels, float &mx, float &my) {
  pixels = 0;
  const int w = fb->width, h = fb->height;
  const long r2min = (long)RAIO_MIN * RAIO_MIN;
  const long r2max = (long)RAIO_MAX * RAIO_MAX;

  long somaX = 0, somaY = 0;
  int n = 0;

  for (int y = 0; y < h; y += 2) {            // pula de 2 em 2 para ser mais rápido
    for (int x = 0; x < w; x += 2) {
      int dx = x - CX, dy = y - CY;
      long d2 = (long)dx * dx + (long)dy * dy;
      if (d2 < r2min || d2 > r2max) continue; // fora do anel: ignora

      uint8_t v = fb->buf[y * w + x];
      bool marcado = MARCA_ESCURA ? (v < LIMIAR) : (v > LIMIAR);
      if (marcado) { somaX += dx; somaY += dy; n++; }
    }
  }

  pixels = n;
  if (n < MIN_PIXELS) return false;

  // Centro de massa da marca, relativo ao centro de rotação
  mx = (float)somaX / n;
  my = (float)somaY / n;

  // Na imagem, Y cresce para baixo, então "cima" = -my.
  // atan2(x, cima) dá 0° para cima e cresce no sentido horário.
  float a = atan2f(mx, -my) * 180.0f / PI;
  a += AJUSTE_ANGULO;
  while (a < 0)    a += 360;
  while (a >= 360) a -= 360;

  angulo = a;
  return true;
}

// Captura um quadro e analisa (usado pelo loop)
bool medirAngulo(float &angulo, int &pixels) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { pixels = 0; return false; }
  float mx, my;
  bool ok = analisarFrame(fb, angulo, pixels, mx, my);
  esp_camera_fb_return(fb);
  return ok;
}

// Arredonda o ângulo para 0..3 (cada passo = 90 graus)
int angulo_para_posicao(float a) {
  return ((int)((a + 45.0f) / 90.0f)) % 4;
}

// ---------- Comunicação com a ESP32 do motor (ESP-NOW) ----------

// Média de ângulos considerando que 359° e 1° são vizinhos (ignora NAN)
bool mediaCircular(const float* v, int n, float &saida) {
  float s = 0, c = 0;
  int k = 0;
  for (int i = 0; i < n; i++) {
    if (isnan(v[i])) continue;
    float r = v[i] * PI / 180.0f;
    s += sinf(r);
    c += cosf(r);
    k++;
  }
  if (k < 2) return false;                    // precisa de pelo menos 2 leituras válidas
  float a = atan2f(s, c) * 180.0f / PI;
  if (a < 0) a += 360;
  saida = a;
  return true;
}

bool iniciarEspNow() {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ENDERECO_BROADCAST, 6);
  peer.channel = 0;                           // 0 = usa o canal atual do Wi-Fi
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void enviarAngulo(bool valido, float angulo, int pixels) {
  static uint16_t seq = 0;
  PacoteAngulo p;
  p.magic  = 0xA5;
  p.valido = valido ? 1 : 0;
  p.seq    = seq++;
  p.angulo = angulo;
  p.pixels = (uint16_t)pixels;
  esp_now_send(ENDERECO_BROADCAST, (uint8_t*)&p, sizeof(p));
}

// ---------- Desenho sobre a imagem (para a rota /foto) ----------

// Fonte 5x7 minúscula: só o que precisamos para escrever ângulos.
// Cada caractere = 5 colunas; em cada coluna o bit 0 é a linha de cima.
static const char* CARACTERES = "0123456789.-+`";   // ` = símbolo de grau
static const uint8_t FONTE[14][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
  {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
  {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
  {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
  {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
  {0x00, 0x60, 0x60, 0x00, 0x00},  // .
  {0x08, 0x08, 0x08, 0x08, 0x08},  // -
  {0x08, 0x08, 0x3E, 0x08, 0x08},  // +
  {0x00, 0x06, 0x09, 0x09, 0x06},  // grau
};

static int indiceGlifo(char c) {
  for (int i = 0; CARACTERES[i]; i++) if (CARACTERES[i] == c) return i;
  return -1;                                   // espaço ou desconhecido: não desenha
}

// Pinta um retângulo (com proteção contra sair da imagem)
static void preencher(camera_fb_t* fb, int x, int y, int w, int h, uint8_t cor) {
  const int W = fb->width, H = fb->height;
  for (int j = y; j < y + h; j++) {
    if (j < 0 || j >= H) continue;
    for (int i = x; i < x + w; i++) {
      if (i < 0 || i >= W) continue;
      fb->buf[j * W + i] = cor;
    }
  }
}

// Um ponto de espessura "esp" (1 = pixel único, 3 = bloco 3x3)
static void ponto(camera_fb_t* fb, int x, int y, uint8_t cor, int esp) {
  int m = esp / 2;
  preencher(fb, x - m, y - m, esp, esp, cor);
}

// Linha a partir do centro em direção a um ângulo "bruto" da imagem
// (graus, horário, 0 = cima), do raio r0 até r1. passo > 1 faz linha pontilhada.
static void linhaPolar(camera_fb_t* fb, float graus, float r0, float r1,
                       uint8_t cor, int esp, int passo) {
  float r = graus * PI / 180.0f;
  float s = sinf(r), c = cosf(r);
  for (float d = r0; d <= r1; d += passo)
    ponto(fb, CX + (int)lroundf(d * s), CY - (int)lroundf(d * c), cor, esp);
}

static void circulo(camera_fb_t* fb, int cx, int cy, int raio, uint8_t cor, int esp) {
  for (int a = 0; a < 720; a++) {
    float r = a * 0.5f * PI / 180.0f;
    ponto(fb, cx + (int)lroundf(raio * cosf(r)), cy + (int)lroundf(raio * sinf(r)), cor, esp);
  }
}

// Texto branco sobre caixa preta. (x,y) = canto superior esquerdo.
static void texto(camera_fb_t* fb, int x, int y, const char* s, int escala) {
  int n = strlen(s);
  int larg = n * 6 * escala - escala;
  preencher(fb, x - 2, y - 2, larg + 4, 7 * escala + 4, 0);
  for (int i = 0; i < n; i++) {
    int g = indiceGlifo(s[i]);
    if (g < 0) continue;
    for (int col = 0; col < 5; col++) {
      uint8_t bits = FONTE[g][col];
      for (int lin = 0; lin < 7; lin++)
        if (bits & (1 << lin))
          preencher(fb, x + (i * 6 + col) * escala, y + lin * escala, escala, escala, 255);
    }
  }
}

// Texto centralizado em (cx, cy), sem sair da imagem
static void textoCentrado(camera_fb_t* fb, int cx, int cy, const char* s, int escala) {
  const int W = fb->width, H = fb->height;
  int larg = strlen(s) * 6 * escala - escala;
  int alt  = 7 * escala;
  int x = constrain(cx - larg / 2, 3, W - larg - 3);
  int y = constrain(cy - alt / 2, 3, H - alt - 3);
  texto(fb, x, y, s, escala);
}

// Anéis, círculo de 8 cm, cruz do centro e linhas de ângulo. Os ângulos "bruto"
// descontam o AJUSTE_ANGULO, para que o "0" fique onde o código realmente considera 0.
static void desenharGuias(camera_fb_t* fb, uint8_t cor, int esp) {
  circulo(fb, CX, CY, RAIO_MIN, cor, esp);
  circulo(fb, CX, CY, RAIO_MAX, cor, esp);
  circulo(fb, CX, CY, RAIO_MARCA, cor, 1);   // referência: onde a marca deve passar (8 cm)
  for (int i = -6; i <= 6; i++) {
    ponto(fb, CX + i, CY, cor, esp);
    ponto(fb, CX, CY + i, cor, esp);
  }
  for (int k = 0; k < 4; k++) {
    // linha cheia: 0, 90, 180, 270
    linhaPolar(fb, k * 90 - AJUSTE_ANGULO, RAIO_MIN, RAIO_MAX, cor, esp, 1);
    // pontilhada: 45, 135, 225, 315 (onde a posição "vira")
    linhaPolar(fb, k * 90 + 45 - AJUSTE_ANGULO, RAIO_MIN, RAIO_MAX, cor, esp, 4);
  }
}

// Linha do centro até a marca + círculo em volta dela
static void desenharMarca(camera_fb_t* fb, float mx, float my, uint8_t cor, int esp) {
  float bruto = atan2f(mx, -my) * 180.0f / PI;
  float comp  = sqrtf(mx * mx + my * my);
  linhaPolar(fb, bruto, 0, comp, cor, esp, 1);
  circulo(fb, CX + (int)lroundf(mx), CY + (int)lroundf(my), 6, cor, esp);
}

// ---------- Servidor web ----------
static esp_err_t rotaPosicao(httpd_req_t* req) {
  int fase = proximaFase;
  char json[320];
  snprintf(json, sizeof(json),
           "{\"fase_mais_proxima\":%d,\"nome\":\"%s\",\"angulo\":%.1f,"
           "\"desvio\":%.1f,\"detectado\":%s,"
           "\"posicao\":%d,\"pixels\":%d}",
           fase,
           fase >= 0 ? NOMES[fase] : "sem leitura",
           (float)ultimoAngulo,
           (float)desvioFase,
           detectadoAgora ? "true" : "false",
           (int)posicaoEstavel,
           (int)ultimosPixels);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Página que se atualiza sozinha mostrando a fase mais próxima
static const char PAGINA[] = R"rawliteral(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Fase do objeto</title>
<style>
body{font-family:sans-serif;text-align:center;margin:0;padding:16px}
#fase{font-size:2.2em;font-weight:bold;margin:12px 0}
#info{font-size:1.1em;margin-bottom:16px;min-height:1.4em}
.f{display:inline-block;width:22%;min-width:90px;padding:12px 0;margin:4px;
   border:2px solid #888;border-radius:10px;font-size:1em;opacity:.45}
.f.ativa{opacity:1;background:#2e7d32;color:#fff;border-color:#2e7d32}
</style></head><body>
<h3>Fase mais pr&oacute;xima</h3>
<div id="fase">...</div>
<div id="info"></div>
<div>
<span class="f" id="f0">Nova<br><small>0&deg;</small></span><span class="f" id="f1">Crescente<br><small>90&deg;</small></span><span class="f" id="f2">Cheia<br><small>180&deg;</small></span><span class="f" id="f3">Minguante<br><small>270&deg;</small></span>
</div>
<p><a href="/foto">Foto</a> | <a href="/posicao">JSON</a></p>
<script>
async function atualizar(){
  try{
    const d=await (await fetch('/posicao')).json();
    for(let k=0;k<4;k++)
      document.getElementById('f'+k).className='f'+(d.detectado&&d.fase_mais_proxima==k?' ativa':'');
    const f=document.getElementById('fase'), i=document.getElementById('info');
    if(d.fase_mais_proxima<0){f.textContent='Sem leitura';i.textContent='';return;}
    f.textContent=d.nome;
    i.textContent=(d.detectado?'':'(marca perdida - ultimo valor) ')+
      '\u00C2ngulo: '+d.angulo.toFixed(1)+'\u00B0 | Desvio: '+
      (d.desvio>=0?'+':'')+d.desvio.toFixed(1)+'\u00B0 | Pixels: '+d.pixels;
  }catch(e){}
}
setInterval(atualizar,300);atualizar();
</script></body></html>)rawliteral";

static esp_err_t rotaInicio(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, PAGINA, HTTPD_RESP_USE_STRLEN);
}

// Foto em cinza com anel, ângulos e marca detectada desenhados
static esp_err_t rotaFoto(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

  // 1) Analisa ANTES de desenhar (os desenhos não podem entrar na detecção)
  float ang = 0, mx = 0, my = 0;
  int px = 0;
  bool achou = analisarFrame(fb, ang, px, mx, my);

  // 2) Desenha em duas passadas: contorno preto grosso e depois linha branca fina.
  //    Assim as marcações aparecem tanto em fundo claro quanto escuro.
  for (int passada = 0; passada < 2; passada++) {
    uint8_t cor = (passada == 0) ? 0 : 255;
    int esp     = (passada == 0) ? 3 : 1;
    desenharGuias(fb, cor, esp);
    if (achou) desenharMarca(fb, mx, my, cor, esp);
  }

  // 3) Números dos ângulos POR DENTRO do anel (fora dele não cabe na imagem)
  char buf[24];
  for (int k = 0; k < 4; k++) {
    float bruto = k * 90 - AJUSTE_ANGULO;
    float r = bruto * PI / 180.0f;
    float d = RAIO_MIN - 18;
    snprintf(buf, sizeof(buf), "%d`", k * 90);
    textoCentrado(fb, CX + (int)lroundf(d * sinf(r)), CY - (int)lroundf(d * cosf(r)), buf, 2);
  }

  // 4) Leitura no canto: ângulo medido, posição arredondada e desvio
  if (achou) {
    int alvo = angulo_para_posicao(ang) * 90;
    float desvio = ang - alvo;
    if (desvio > 180)  desvio -= 360;
    if (desvio < -180) desvio += 360;

    snprintf(buf, sizeof(buf), "%.1f`", ang);
    texto(fb, 5, 5, buf, 2);
    snprintf(buf, sizeof(buf), "%d`", alvo);
    texto(fb, 5, 25, buf, 2);
    snprintf(buf, sizeof(buf), "%+.1f", desvio);
    texto(fb, 5, 45, buf, 2);
  } else {
    texto(fb, 5, 5, "---", 2);                // marca não encontrada
  }

  uint8_t* jpg = NULL;
  size_t len = 0;
  bool ok = frame2jpg(fb, 90, &jpg, &len);    // converte cinza -> JPEG
  esp_camera_fb_return(fb);
  if (!ok) { httpd_resp_send_500(req); return ESP_FAIL; }

  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t r = httpd_resp_send(req, (const char*)jpg, len);
  free(jpg);
  return r;
}

void iniciarServidor() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t uPos  = { "/posicao", HTTP_GET, rotaPosicao, NULL };
  httpd_uri_t uFoto = { "/foto",    HTTP_GET, rotaFoto,    NULL };
  httpd_uri_t uRaiz = { "/",        HTTP_GET, rotaInicio,  NULL };
  if (httpd_start(&servidor, &config) == ESP_OK) {
    httpd_register_uri_handler(servidor, &uPos);
    httpd_register_uri_handler(servidor, &uFoto);
    httpd_register_uri_handler(servidor, &uRaiz);
  }
}

// ---------- setup / loop ----------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, LOW);   // dica: luz constante ajuda MUITO a detecção

  if (!iniciarCamera()) {
    Serial.println("Falha na câmera.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nIP: http://");
  Serial.println(WiFi.localIP());
  Serial.printf(">>> CANAL WI-FI = %d  (use este valor em CANAL_WIFI no sketch do motor)\n", WiFi.channel());

  if (!iniciarEspNow()) Serial.println("Falha ao iniciar ESP-NOW.");

  iniciarServidor();
}

void loop() {
  // Votação: guarda as últimas 5 leituras e usa a mais frequente.
  // Isso evita que uma leitura ruim (sombra, reflexo) mude a posição.
  static int historico[5] = {-1, -1, -1, -1, -1};
  static int idx = 0;
  static int ultimaImpressa = -2;

  float ang = 0;
  int px = 0;
  int leitura = -1;
  bool achou = medirAngulo(ang, px);
  detectadoAgora = achou;
  if (achou) {
    leitura = angulo_para_posicao(ang);       // fase mais próxima (0-3)
    ultimoAngulo = ang;

    float d = ang - leitura * 90.0f;          // quanto falta/passou em relação a ela
    if (d > 180)  d -= 360;
    if (d < -180) d += 360;
    desvioFase = d;
    proximaFase = leitura;
  }
  ultimosPixels = px;

  // Envia o ângulo ao motor: média circular das últimas 3 leituras (suaviza ruído)
  static float bufAng[3] = {NAN, NAN, NAN};
  static int iAng = 0;
  bufAng[iAng] = achou ? ang : NAN;
  iAng = (iAng + 1) % 3;
  float angFiltrado = 0;
  bool filtroOk = mediaCircular(bufAng, 3, angFiltrado);
  enviarAngulo(achou && filtroOk, angFiltrado, px);

  historico[idx] = leitura;
  idx = (idx + 1) % 5;

  int votos[4] = {0, 0, 0, 0};
  for (int i = 0; i < 5; i++) if (historico[i] >= 0) votos[historico[i]]++;
  int melhor = -1, max = 2;                   // precisa de pelo menos 3 votos de 5
  for (int p = 0; p < 4; p++) if (votos[p] > max) { max = votos[p]; melhor = p; }
  posicaoEstavel = melhor;

  if (melhor != ultimaImpressa) {
    ultimaImpressa = melhor;
    Serial.printf("Posição: %s | ângulo %.1f | pixels %d\n",
                  melhor >= 0 ? NOMES[melhor] : "nao detectado", (float)ultimoAngulo, px);
  }
  delay(40);                                  // leituras rápidas para o controle
}
