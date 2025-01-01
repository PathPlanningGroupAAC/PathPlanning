#include <iostream>
#include <iomanip>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;

#include <vector>

#define PI 3.14159265359


float calculateAngle(const glm::vec2& point1, const glm::vec2& point2) {
    
    const float dotProduct = glm::dot(point1, point2);
    
    
    const float magnitude1 = glm::length(point1);
    const float magnitude2 = glm::length(point2);

    
    if (magnitude1 == 0.0f || magnitude2 == 0.0f) {
        throw std::invalid_argument("Uno dei vettori ha lunghezza zero.");
    }

    
    float cosTheta = dotProduct / (magnitude1 * magnitude2);
    cosTheta = glm::clamp(cosTheta, -1.0f, 1.0f);

    return glm::acos(cosTheta);
}

/*

    Formato punti:
    std::vector<glm::vec2> left_points;
    std::vector<glm::vec2> right_points;

*/

// all_points tutti i punti che vede la macchina, punti_scelti i punti già analizzati e presi
std::vector<glm::vec2> trova_adiacenti(const std::vector<glm::vec2>& all_points, float raggio_di_ricerca, const std::vector<glm::vec2>& punti_scelti, float angolo_di_ricerca);

int main()
{
    glm::vec2 A = glm::vec2(0,1);
    glm::vec2 B = glm::vec2(-1,0);

    float dist = glm::distance(A,B);
    std::cout << "dist = " << dist << '\n';

    float angle = calculateAngle(A,B);
    std::cout << "angle = " << angle << '\n';

    plt::plot({1,3,2,4});
    plt::show();
    return 0;
}

// Escludi gli adiacenti che non soddisfano angolo e distanza
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
            float angolo = CalculateAngle(vettore_ultimo_segmento, vettore_cono_filtrato);

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
