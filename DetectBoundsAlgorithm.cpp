#include "DetectBoundsAlgorithm.h"
#include <optional>

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
    glm::vec2 lp = get_left_right_points(veichle, cones_blue, dmax); //lp
    glm::vec2 lp2 = trova_punto_con_angolo(lp, vettore_direzione, cones_blue); //lp2
    
    glm::vec2 rp = get_left_right_points(veichle, cones_yellow, dmax); //rp
    glm::vec2 rp2 = trova_punto_con_angolo(rp, vettore_direzione, cones_yellow); //rp2

    punti_finali_left.push_back(lp);
    punti_finali_left.push_back(lp2);

    punti_finali_right.push_back(rp);
    punti_finali_right.push_back(rp2);

    // Inizio algoritmo
    const int max_points_blue = cones_blue.size();
    int i = 0;
    while(i < max_points_blue)
    {
        std::vector<glm::vec2> adiacenti = trova_adiacenti(cones_blue, dmax, punti_finali_left, angolo_max_ricerca);

        if (adiacenti.empty())
        {
            //TODO: Restituire qualcosa?
            std::cout << "ERROR\n";
            return;
        }

        punti_finali_left = nvd(punti_finali_left, adiacenti, grado_spline);

        i++;
    }

    const int max_points_yellow = cones_yellow.size();
    i = 0;
    while (i < max_points_yellow)
    {
        std::vector<glm::vec2> adiacenti = trova_adiacenti(cones_yellow, dmax, punti_finali_right, angolo_max_ricerca);

        if (adiacenti.empty())
        {
            //TODO: Restituire qualcosa?
            std::cout << "ERROR\n";
            return;
        }

        punti_finali_right = nvd(punti_finali_right, adiacenti, grado_spline);

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
    spapi(grado_spline, x, spline_x);
    spapi(grado_spline, y, spline_y);


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

glm::vec2 trova_punto_con_angolo(const glm::vec2& punto_iniziale, glm::vec2& vett_direzione, const std::vector<glm::vec2>& matrice_punti) {

    // Inizializzare variabili
    glm::vec2 norma_direzione = glm::normalize(vett_direzione); // Calcolare la norma del vettore di direzione
    float min_angolo = std::numeric_limits<float>::infinity();  // Angolo minimo inizializzato a infinito
    std::optional<glm::vec2> punto_trovato;                     // Punto trovato (inizialmente vuoto)

    // Scorrere ogni punto nella matrice
    for (const auto& punto : matrice_punti) {

        // Calcolare il vettore dal punto iniziale al punto corrente
        const glm::vec2 vettore_punto = punto - punto_iniziale;

        // Calcolare l'angolo tra il vettore di direzione e il vettore dal punto iniziale al punto corrente
        const float angle = calculateAngle(vett_direzione, vettore_punto);

        // Se l'angolo è minore del minimo trovato finora, aggiorna
        if (angle < min_angolo)
        {
            min_angolo = angle;
            punto_trovato = punto;
        }

    }

    // Se non è stato trovato alcun punto, restituisci un messaggio di errore
    if (!punto_trovato.has_value())
    {
        throw std::invalid_argument("Nessun punto trovato con angolo minore rispetto alla direzione.");
    }

    return punto_trovato.value();
}

glm::vec2 get_left_right_points(const glm::vec2& pos, const std::vector<glm::vec2>& points, float max_distance)
{
    std::vector<float> distances;
    for (const auto& point : points) {
        distances.push_back(glm::distance(point, pos));
    }

    std::vector<float> valid_distances;
    std::vector<glm::vec2> valid_points;     // Punti validi entro il range
    int i = 0;
    for (const auto& dist : distances) {
        if (dist >= 2.5 && dist <= max_distance)
        {
            valid_points.push_back(points[i]);
            valid_distances.push_back(distances[i]);
        }
        i++;
    }

    // Se nessun punto è valido, lancia un'eccezione
    if (valid_points.empty()) {
        throw std::runtime_error("Nessun punto valido trovato");
    }

    // Trova il punto con la distanza minima
    float min_distance = std::numeric_limits<float>::infinity();
    size_t min_idx = 0;
    for (size_t i = 0; i < valid_distances.size(); ++i) {
        if (valid_distances[i] < min_distance) {
            min_distance = valid_distances[i];
            min_idx = i;
        }
    }

    return valid_points[min_idx];
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

void spapi(int grado, const std::vector<double>& valori, std::vector<double>& spline)
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

void spapi2(int grado, const std::vector<double>& valori, std::vector<double>& spline) {
    if (grado != 2) {
        throw std::invalid_argument("Grado non valido: atteso 2");
    }

    int n = valori.size();
    if (n < 2) {
        throw std::invalid_argument("Ci devono essere almeno due punti.");
    }

    spline.resize(n);

    // Calcolare i coefficienti della spline quadratica
    // Supponiamo che i punti siano (x0, y0), (x1, y1), ..., (xn-1, yn-1)

    // I coefficienti della spline quadratica sono determinati risolvendo un sistema di equazioni lineari
    std::vector<double> a(n - 1), b(n - 1), c(n - 1);

    // Passo 1: Calcoliamo la matrice di sistema
    for (int i = 0; i < n - 1; ++i) {
        double dx = valori[i + 1] - valori[i]; // distanza tra i punti
        a[i] = (valori[i + 1] - valori[i]) / (dx * dx);
        b[i] = 2.0 * (valori[i] - valori[i + 1]);
        c[i] = valori[i];  // c[i] rappresenta il valore della spline a i-th punto
    }

    // Step 2: Definire la spline utilizzando i coefficienti calcolati
    for (int i = 0; i < n - 1; ++i) {
        spline[i] = a[i] * valori[i] * valori[i] + b[i] * valori[i] + c[i];
    }

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

double fnval2(const std::vector<double>& spline, double t) {
    int n = spline.size();
    if (t <= 0) return spline[0];             // Estremo sinistro
    if (t >= 1) return spline[n - 1];         // Estremo destro

    // Trova il segmento corretto
    int idx = static_cast<int>(t * (n - 1));
    double alpha = t * (n - 1) - idx;

    // Interpolazione lineare tra spline[idx] e spline[idx + 1]
    return (1 - alpha) * spline[idx] + alpha * spline[idx + 1];
}