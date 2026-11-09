# 2D-Rigid-Body-Dynamik

Die erste Modellstufe basiert auf Newton-Euler-Dynamik. Aerodynamik, Aktuatorverzögerungen und Kontakte sind bewusst nicht enthalten.

## Schubvektor

Im Body Frame gilt

\[
\mathbf F_T^B = T
\begin{bmatrix}
\sin\delta\\
0\\
\cos\delta
\end{bmatrix}.
\]

Mit der Rotation des Fahrzeugs um \(\theta\) ergibt sich im World Frame

\[
\mathbf F_T^W = T
\begin{bmatrix}
\sin(\theta+\delta)\\
0\\
\cos(\theta+\delta)
\end{bmatrix}.
\]

Die Schubrichtung relativ zur Welt wird unter den festgelegten Vorzeichenkonventionen also durch \(\theta+\delta\) bestimmt.

## Translation

Newtons zweites Gesetz lautet

\[
m\ddot{\mathbf p}=\mathbf F_T+\mathbf F_g.
\]

Daraus folgt

\[
\ddot p_x = \frac{T}{m}\sin(\theta+\delta)
\]

und

\[
\ddot p_z = \frac{T}{m}\cos(\theta+\delta)-g.
\]

## Drehmoment durch den Gimbal

Der Vektor vom Schwerpunkt zum Schubangriffspunkt ist

\[
\mathbf r^B =
\begin{bmatrix}
0\\
0\\
-l
\end{bmatrix}.
\]

Das Moment folgt aus

\[
\boldsymbol\tau = \mathbf r \times \mathbf F_T.
\]

Damit ergibt sich

\[
\boldsymbol\tau^B =
\begin{bmatrix}
0\\
-lT\sin\delta\\
0
\end{bmatrix}
\]

und somit

\[
\tau_y=-lT\sin\delta.
\]

Das Minuszeichen ist eine physikalische Folge der definierten positiven Richtungen. Bei \(\delta>0\) wird der unterhalb des Schwerpunktes liegende Angriffspunkt nach \(+x_B\) gedrückt; dadurch dreht die Nase in negative \(\theta\)-Richtung.

## Rotation

Für die 2D-Rotation genügt

\[
I_y\dot\omega=\tau_y.
\]

Daraus folgt

\[
\dot\omega=-\frac{lT}{I_y}\sin\delta.
\]

## Vollständige Zustandsableitung

\[
\boxed{
\dot{\mathbf x}=
\begin{bmatrix}
v_x\\
v_z\\
\dfrac{T}{m}\sin(\theta+\delta)\\
\dfrac{T}{m}\cos(\theta+\delta)-g\\
\omega\\
-\dfrac{lT}{I_y}\sin\delta
\end{bmatrix}
}
\]

Diese Gleichung ist die mathematische Referenz für `Hopper2DDynamics.derivative()`.
