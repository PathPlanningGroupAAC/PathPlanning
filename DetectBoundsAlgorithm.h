#pragma once

#include <iostream>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

/*
   _   _                  _ _   _               
  /_\ | | __ _  ___  _ __(_) |_| |__  _ __ ___  
 //_\\| |/ _` |/ _ \| '__| | __| '_ \| '_ ` _ \ 
/  _  \ | (_| | (_) | |  | | |_| | | | | | | | |
\_/ \_/_|\__, |\___/|_|  |_|\__|_| |_|_| |_| |_|
         |___/                                      
*/

/**
 *  Punto di ingresso dell'algoritmo.
 */
void begin_frame(const std::vector<glm::vec2>& cones_blue, const std::vector<glm::vec2>& cones_yellow, glm::vec2 veichle, glm::vec2 vettore_direzione, std::vector<glm::vec2>& punti_finali_left, std::vector<glm::vec2>& punti_finali_right);

/**
 *  Cerca tra gli adiacenti il prossimo punto.
 *
 *  Parametri:
 *  punti_correnti          -> Punti validi.
 *  adiacenti_correnti      -> Adiacenti restituiti da 'trova_adicenti'.
 *  grado_spline            -> Grado spline per l'interpolazione (supporto solo per quelle di 2a grado).
 */
std::vector<glm::vec2> nvd(const std::vector<glm::vec2>& punti_correnti, const std::vector<glm::vec2>& adiacenti_correnti, int grado_spline);

/**
 *  Trova gli adiacenti all'ultimo punto escludendo gli adiacenti che non soddisfano 
 *  i constraints (angolo e distanza).
 *  
 *  Parametri:
 *  all_points        -> Tutti i punti che vede la macchina.
 *  punti_scelti      -> Punti già analizzati e presi.
 *  angolo_di_ricerca -> Angolo.
 */
std::vector<glm::vec2> trova_adiacenti(const std::vector<glm::vec2>& all_points, float raggio_di_ricerca, const std::vector<glm::vec2>& punti_scelti, float angolo_di_ricerca);


/*
              _   _     
  /\/\   __ _| |_| |__  
 /    \ / _` | __| '_ \ 
/ /\/\ \ (_| | |_| | | |
\/    \/\__,_|\__|_| |_|
                        
*/

/**
 *  Calcola l'angolo tra due punti.
 *  
 *  Parametri:
 *  point1    -> Primo punto.
 *  point2    -> Secondo Punto.
 */
float calculateAngle(const glm::vec2& point1, const glm::vec2& point2);

/**
 *  Funzione per interpolare spline quadratica.
 *
 *  Parametri:
 *  grado     -> Il grado della spline (deve essere 2; altrimenti, è ignorato per questa implementazione).
 *  t         -> Un vettore di valori normalizzati (0 a 1) che rappresenta punti.
 *  valori    -> I valori originali da interpolare.
 *  spline    -> Un vettore in uscita che conterrà i valori interpolati della spline.
 */
void spapi(int grado, const std::vector<double>& t, const std::vector<double>& valori, std::vector<double>& spline);

 /**
  *  Funzione per interpolare spline quadratica.
  *
  *  Parametri:
  *  spline    -> Primo punto.
  *  t         -> Secondo Punto.
  */
double fnval(const std::vector<double>& spline, double t);