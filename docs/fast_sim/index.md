# Fast Simulator

Der Fast Simulator ist das mathematisch transparente Simulations-Backend des Rocket-Hopper-Projekts.

Seine Kernaufgabe ist die Abbildung

\[
(x, u, p) \rightarrow \dot{x}
\]

und, nach Ergänzung eines Integrators,

\[
(x_k, u_k, p) \rightarrow x_{k+1}.
\]

## Version 0

Die erste Modellstufe betrachtet einen zweidimensionalen Starrkörper in der Welt-\(x\)-\(z\)-Ebene. Der Hopper besitzt drei Freiheitsgrade:

- horizontale Translation,
- vertikale Translation,
- Rotation um die \(y\)-Achse.

Der Starrkörper erhält als physikalische Inputs den tatsächlich erzeugten Schub \(T\) und den tatsächlich erreichten Gimbalwinkel \(\delta\).

## Bewusst nicht Teil der ersten Dynamik

- Servozeitkonstanten,
- Gimbal-Rate-Limits,
- Motordynamik,
- Batterie,
- Aerodynamik,
- Sensoren,
- Boden- und Landebeinkontakte.

Diese Effekte werden später als eigene Komponenten ergänzt, ohne die grundlegende Starrkörperdynamik zu vermischen.
