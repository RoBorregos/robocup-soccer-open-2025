#include "DataFilter.h"

#include <Arduino.h>

DataFilter::DataFilter(int tamanoVentana) {
  _tamanoVentana = tamanoVentana;
  _valores = new float[tamanoVentana];
  reiniciar();
}

DataFilter::~DataFilter() {
  delete[] _valores;
}

float DataFilter::addValue(float nuevoValor) {
  if(_lleno) {
    _suma -= _valores[_indice];  // Resta el valor más antiguo
  }
  
  _valores[_indice] = nuevoValor; // Almacena el nuevo valor
  _suma += nuevoValor;           // Suma el nuevo valor
  
  _indice = (_indice + 1) % _tamanoVentana; // Avanza el índice
  
  if(!_lleno && _indice == 0) {
    _lleno = true; // La ventana se ha llenado por primera vez
  }
  
  return obtainProm();
}

float DataFilter::obtainProm() const {
  int elementos = _lleno ? _tamanoVentana : _indice;
  return elementos > 0 ? _suma / elementos : 0;
}

void DataFilter::reset() {
  _indice = 0;
  _suma = 0;
  _lleno = false;
  for(int i = 0; i < _tamanoVentana; i++) {
    _valores[i] = 0;
  }
}