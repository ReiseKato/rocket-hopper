# Koordinatensystem und Vorzeichen

Diese Seite ist die verbindliche Referenz für die 2D-Konventionen. Vorzeichen in den Bewegungsgleichungen dürfen nur ausgehend von diesen Definitionen interpretiert werden.

## World Frame \(W\)

Die Bewegung findet in der \(x\)-\(z\)-Ebene statt:

- \(+x_W\): horizontal nach rechts,
- \(+z_W\): vertikal nach oben,
- \(+y_W\): in die Bildebene hinein.

Damit bleibt das zugrunde liegende 3D-Koordinatensystem rechtshändig.

Die Gewichtskraft ist

\[
\mathbf F_g^W =
\begin{bmatrix}
0\\
0\\
-mg
\end{bmatrix}.
\]

## Body Frame \(B\)

Der Ursprung des Body Frames liegt im Schwerpunkt (Center of Gravity, CG). Das Frame dreht sich mit dem Fahrzeug.

- \(+z_B\): entlang der Längsachse vom Triebwerk zur Nase,
- \(+x_B\): bei aufrechtem Fahrzeug nach rechts,
- \(+y_B\): in die Bildebene hinein.

## Fahrzeugwinkel \(\theta\)

\[
\theta = 0
\]

bedeutet einen aufrechten Hopper. Ein positiver Winkel

\[
\theta > 0
\]

bedeutet: **Die Nase kippt nach rechts.**

Die Winkelgeschwindigkeit ist

\[
\omega = \dot\theta.
\]

## Triebwerksposition

Der Schubangriffspunkt liegt im ersten Modell auf der Längsachse um den positiven Abstand \(l\) unterhalb des Schwerpunkts:

\[
\mathbf r_{CG\rightarrow E}^B =
\begin{bmatrix}
0\\
0\\
-l
\end{bmatrix},
\qquad l \ge 0.
\]

## Gimbalwinkel \(\delta\)

\[
\delta = 0
\]

bedeutet Schub entlang \(+z_B\).

\[
\delta > 0
\]

bedeutet: **Der tatsächliche Schubvektor wird in Richtung \(+x_B\) geneigt.**

Diese Definition bezieht sich auf den Kraftvektor auf das Fahrzeug, nicht auf einen späteren mechanischen Servowinkel.

Damit gilt im Body Frame

\[
\mathbf F_T^B = T
\begin{bmatrix}
\sin\delta\\
0\\
\cos\delta
\end{bmatrix}.
\]
