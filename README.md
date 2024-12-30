# Prerequisiti
- Python
- CMake
- MatPlotLib (Una volta scaricato python fai: `pip install matplotlib`)
> Se stai su linux ti serve anche scaricare `python3-dev`. Puoi usare apt `sudo apt install python3-dev`

# Setup iniziale
Apri il terminale dentro la cartella del progetto.
```
mkdir build
cd build
cmake ..
```

# Compilazione
## Windows
Da dentro la cartella build:
```
cmake --build .
```
> **ATTENZIONE:** In caso di errore che riguarda 'python3.x_d.lib' fai `cmake --build . --config RELEASE`

## Linux
Da dentro la cartella build:
```
cmake --build .
```

# Esecuzione
## Windows
Da dentro la cartella build:
```
.\Debug\Bounds.exe
```
> **ATTENZIONE:** Se hai fatto `cmake --build . --config RELEASE` allora fai `.\Release\Bounds.exe`

## Linux
Da dentro la cartella build:
```
./Bounds
```