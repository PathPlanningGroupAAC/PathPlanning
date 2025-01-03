#include "DetectBoundsAlgorithm.h"

#define PI 3.14159265359

/*
   _   _                  _ _   _               
  /_\ | | __ _  ___  _ __(_) |_| |__  _ __ ___  
 //_\\| |/ _` |/ _ \| '__| | __| '_ \| '_ ` _ \ 
/  _  \ | (_| | (_) | |  | | |_| | | | | | | | |
\_/ \_/_|\__, |\___/|_|  |_|\__|_| |_|_| |_| |_|
         |___/                                      
*/

void begin_frame(const std::vector<glm::vec2>& cones_blue, const std::vector<glm::vec2>& cones_yellow, glm::vec2 veichle, glm::vec2 vettore_direzione, std::vector<glm::vec2>& punti_finali_left, std::vector<glm::vec2>& punti_finali_right)
{
    // Constraints
    const float dmax = 5.5;
    const float angolo_max_ricerca = 1.396;
    const float grado_spline = 2;

    // Primi 2 punti (sx, dx)
    glm::vec2 lp = glm::vec2(22.353429448453898, 42.108144483620897); //lp
    glm::vec2 lp2 = glm::vec2(24.316507170058600, 42.095788661670099); //lp2
    
    glm::vec2 rp = glm::vec2(24.621401660872646, 47.625933776065160); //rp
    glm::vec2 rp2 = glm::vec2(34.735435411001298, 47.530370713741398); //rp2

    punti_finali_left.push_back(lp);
    punti_finali_left.push_back(lp2);

    punti_finali_right.push_back(rp);
    punti_finali_right.push_back(rp2);

    // Inizio algoritmo
    const int max_points = cones_blue.size(); // ASSUNTO che siano della stessa dimensione
    int i = 0;
    while(i < max_points)
    {
        int j = 0;
        while(j < 2)
        {
            std::vector<glm::vec2> adiacenti = trova_adiacenti((j == 0) ? (cones_blue):(cones_yellow), dmax, (j == 0) ? (punti_finali_left):(punti_finali_right), angolo_max_ricerca);
            if(adiacenti.empty())
            {
                //TODO: Restituire qualcosa?
                std::cout << "ERROR\n";
                return;
            }

            if (j == 0)
            {
                punti_finali_left = nvd(punti_finali_left, adiacenti, grado_spline);
            }
            else {
                punti_finali_right = nvd(punti_finali_right, adiacenti, grado_spline);
            }

            j++;
        }
        i++;
    }

    
}

std::vector<glm::vec2> nvd(const std::vector<glm::vec2>& punti_correnti, const std::vector<glm::vec2>& adiacenti_correnti, int grado_spline) {
    // DEBUG ONLY grado
    if (grado_spline != 2)
        throw std::invalid_argument("Grado non valido");

    // Estrai coordinate x e y
    std::vector<double> x, y;
    for (const auto& p : punti_correnti) {
        x.push_back(p.x);
        y.push_back(p.y);
    }

    // Crea vettore t (intervallo normalizzato)
    std::vector<double> t(punti_correnti.size());
    for (size_t i = 0; i < t.size(); ++i) {
        t[i] = static_cast<double>(i) / (t.size() - 1);
    }

    // Calcolo spline quadratiche
    std::vector<double> spline_x, spline_y;
    spapi(grado_spline, t, x, spline_x);
    spapi(grado_spline, t, y, spline_y);

    // Punto finale spline
    double x_end = fnval(spline_x, 1.0);
    double y_end = fnval(spline_y, 1.0);
    glm::vec2 punto_fine(x_end, y_end);

    // Tangente nell'ultimo punto
    double dx = fnval(spline_x, 1.0) - fnval(spline_x, 0.9);
    double dy = fnval(spline_y, 1.0) - fnval(spline_y, 0.9);
    glm::vec2 vettore_tangente(dx, dy);
    vettore_tangente = glm::normalize(vettore_tangente);

    // Calcola distanze
    std::vector<double> distanze(adiacenti_correnti.size());
    for (size_t i = 0; i < adiacenti_correnti.size(); ++i) {
        glm::vec2 cono = adiacenti_correnti[i];
        glm::vec2 vettore_cono = cono - punto_fine;
        double distanza_perpendicolare = std::abs(glm::determinant(glm::dmat2(vettore_tangente, glm::dvec2(vettore_cono)))) / glm::length(vettore_tangente);
        distanze[i] = distanza_perpendicolare;
    }

    // Trova il minimo
    auto min_it = std::min_element(distanze.begin(), distanze.end());
    int indice_minimo = std::distance(distanze.begin(), min_it);
    glm::vec2 cono_scelto = adiacenti_correnti[indice_minimo];

    // Aggiorna punti
    std::vector<glm::vec2> punti_aggiornati = punti_correnti;
    punti_aggiornati.push_back(cono_scelto);

    return punti_aggiornati;
}

std::vector<glm::vec2> trova_adiacenti(const std::vector<glm::vec2>& all_points, float raggio_di_ricerca, const std::vector<glm::vec2>& punti_scelti, float angolo_di_ricerca) {
    float distanza_max = raggio_di_ricerca;
    glm::vec2 ultimo_punto = punti_scelti.back(); 
    glm::vec2 penultimo_punto = punti_scelti[ punti_scelti.size() - 2 ];

    std::vector<glm::vec2> adiacenti;
    bool restart = true;

    while (restart) {
        restart = false;
        for (const auto& punto_attuale : all_points) {
            // Calcolo della distanza
            float distanza = glm::distance(punto_attuale, ultimo_punto);

            // Calcolo angolo
            glm::vec2 vettore_ultimo_segmento = ultimo_punto - penultimo_punto;
            glm::vec2 vettore_cono_filtrato = punto_attuale - ultimo_punto;
            float angolo = calculateAngle(vettore_ultimo_segmento, vettore_cono_filtrato);

            if (distanza <= distanza_max && distanza >= 2.0f && angolo < angolo_di_ricerca) {
                adiacenti.push_back(punto_attuale);
            }
        }

        // Aumenta dmax e cerca di nuovo se non hai trovato adiacenti (fino a un massimo di 8m)
        if (adiacenti.empty() && distanza_max < 8.5f) {
            restart = true;
            distanza_max += 0.5f;  // Aumenta la distanza massima
        }
    }
    return adiacenti;
}

/*
              _   _     
  /\/\   __ _| |_| |__  
 /    \ / _` | __| '_ \ 
/ /\/\ \ (_| | |_| | | |
\/    \/\__,_|\__|_| |_|
                        
*/

float calculateAngle(const glm::vec2& point1, const glm::vec2& point2) {
    
    const float dotProduct = glm::dot(point1, point2);
    
    const float magnitude1 = glm::length(point1);
    const float magnitude2 = glm::length(point2);
    
    float cosTheta = dotProduct / (magnitude1 * magnitude2);
    cosTheta = glm::clamp(cosTheta, -1.0f, 1.0f);
    
    return glm::acos(cosTheta);
}

void spapi(int grado, const std::vector<double>& t, const std::vector<double>& valori, std::vector<double>& spline)
{
    // DEBUG ONLY grado
    if (grado != 2)
        throw std::invalid_argument("Grado non valido");

    // Interpolazione per spline quadratica (grado = 2)
    int n = valori.size();
    spline.resize(n);

    for (int i = 1; i < n - 1; ++i) {
        spline[i] = (valori[i - 1] + valori[i] + valori[i + 1]) / 3.0;
    }

    // Estremi non hanno punti sufficienti per media, li manteniamo uguali
    spline[0] = valori[0];
    spline[n - 1] = valori[n - 1];
}

double fnval(const std::vector<double>& spline, double t)
{
    // Valutazione spline con interpolazione lineare
    int n = spline.size();
    if (t <= 0) return spline[0];
    if (t >= 1) return spline[n - 1];
    int idx = static_cast<int>(t * (n - 1));
    double alpha = t * (n - 1) - idx;
    return (1 - alpha) * spline[idx] + alpha * spline[idx + 1];
}