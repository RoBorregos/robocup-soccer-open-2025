#ifndef DATAFILTER_H
#define DATAFILTER_H

class DataFilter {
  public:
    // Constructor con tamaño de ventana
    DataFilter(int tamanoVentana);
    
    // Destructor
    ~DataFilter();
    
    // Añade nuevo valor y devuelve la media
    float addValue(float nuevoValor);
    
    // Obtiene la media actual sin añadir valores
    float obtainProm() const;
    
    // Reinicia el filtro
    void reset();
    
  private:
    float* _valores;     // Array dinámico para almacenar valores
    int _tamanoVentana;  // Tamaño de la ventana
    int _indice;         // Índice actual
    float _suma;         // Suma acumulada
    bool _lleno;         // Bandera para indicar si la ventana está llena
};

#endif