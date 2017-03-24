# Kalman Filter implementation Notes

Notation:
  * `x`: System state vector.
  * `z`: Measurement Vector.
  * `H`: Measurement Matrix.
  * `R`: Covariance Matrix.
  * `P`: Also covariance matrix.
  * `F`: Transition matrix.




 The radar information is in the next format:

  | Indicador | ρ | φ | ρ'  |  timestamp  |  d    | d |d|d|
  |:--- | :--- | :--- | :--- | :--- | :--- |
  | R |  8.46642 | 0.0287602  | -3.04035  | 1477010443399637 | 8.6 | 0.25 | -3.00029 | 0


The lidar measurement is linear, then there's no need for Jacobian, but
the radar measurement it isn't then we need to use the Jacobian.
