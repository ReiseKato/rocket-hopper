# Validierung

Die erste Implementierung wird gegen analytisch verständliche Spezialfälle geprüft. Die Tests dienen nicht nur der Softwarequalität, sondern sichern die physikalischen Vorzeichen und Modellannahmen ab.

## Freier Fall

Für

\[
T=0
\]

muss gelten

\[
\dot v_x=0,
\qquad
\dot v_z=-g,
\qquad
\dot\omega=0.
\]

## Idealer Hover

Für

\[
\theta=0,
\qquad
\delta=0,
\qquad
T=mg
\]

muss gelten

\[
\dot v_x=0,
\qquad
\dot v_z=0,
\qquad
\dot\omega=0.
\]

## Geneigtes Fahrzeug ohne Gimbal

Für \(\theta>0\) und \(\delta=0\):

- horizontale Beschleunigung ist positiv,
- die vertikale Schubkomponente ist kleiner als \(T\),
- es entsteht kein Gimbalmoment.

## Positiver Gimbal

Für \(\theta=0\) und \(\delta>0\):

- \(\dot v_x>0\),
- \(\dot\omega<0\).

Dieser Fall prüft insbesondere das Vorzeichen des Drehmoments.

## Symmetrie

Bei sonst identischen Bedingungen müssen \(+\delta\) und \(-\delta\) symmetrische horizontale und rotatorische Reaktionen erzeugen.

## Vertikaler Welt-Schub bei geneigtem Fahrzeug

Für

\[
\delta=-\theta
\]

zeigt der Schub in Weltkoordinaten vertikal. Die Translation kann daher momentan hoverartig sein, während wegen \(\delta\neq0\) trotzdem ein Drehmoment entsteht.
