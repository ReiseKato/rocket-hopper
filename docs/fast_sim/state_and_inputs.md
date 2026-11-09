# Zustand, Inputs und Parameter

## Starrkörperzustand

Der 2D-Zustand ist

\[
\mathbf x =
\begin{bmatrix}
p_x & p_z & v_x & v_z & \theta & \omega
\end{bmatrix}^T.
\]

| Größe | Bedeutung | Einheit |
|---|---|---|
| \(p_x\) | horizontale CG-Position | m |
| \(p_z\) | vertikale CG-Position | m |
| \(v_x\) | horizontale Geschwindigkeit | m/s |
| \(v_z\) | vertikale Geschwindigkeit | m/s |
| \(\theta\) | Fahrzeugwinkel | rad |
| \(\omega\) | Winkelgeschwindigkeit | rad/s |

Intern werden ausschließlich SI-Einheiten verwendet. Winkel werden intern immer in Radiant geführt.

## Physikalische Actuation

Die Starrkörperdynamik erhält

\[
\mathbf u =
\begin{bmatrix}
T & \delta
\end{bmatrix}^T.
\]

- \(T\): tatsächlich auf den Starrkörper wirkender Schub in Newton,
- \(\delta\): tatsächlich erreichter Gimbalwinkel in Radiant.

Die Starrkörperdynamik erhält **keine** PWM-, Throttle- oder Servo-Commands.

## Commands sind nicht Actuation

Langfristig gilt konzeptionell:

\[
\delta_{cmd}
\rightarrow
\text{Gimbalmodell}
\rightarrow
\delta
\]

und entsprechend für den Schub.

Dadurch können reale Aktuatoreigenschaften ergänzt werden, ohne die Newton-Euler-Dynamik zu verändern.

## Starrkörperparameter

Die erste Dynamik benötigt nur:

| Parameter | Bedeutung | Einheit |
|---|---|---|
| \(m\) | Fahrzeugmasse | kg |
| \(I_y\) | Trägheitsmoment um die Body-\(y\)-Achse durch den CG | kg m² |
| \(l\) | Abstand CG → Schubangriffspunkt | m |
| \(g\) | Gravitationsbeschleunigung | m/s² |

Aktuatorgrenzen wie \(T_{max}\), \(\delta_{max}\) oder \(\dot\delta_{max}\) gehören nicht in diese Parametergruppe.

## Numerische Zustandsdarstellung

Die öffentliche Simulationsschnittstelle verwendet `HopperState2D` als kanonische Repräsentation des Fahrzeugzustands.

Numerische Algorithmen wie Integratoren dürfen den Zustand intern vorübergehend in einen NumPy-Vektor umwandeln.

Die kanonische Reihenfolge des Zustandsvektors lautet

\[
x =
\begin{bmatrix}
p_x &
p_z &
v_x &
v_z &
\theta &
\omega
\end{bmatrix}^{T}.
\]

Daraus ergibt sich folgende numerische Reihenfolge:

```text
Index 0: px      [m]
Index 1: pz      [m]
Index 2: vx      [m/s]
Index 3: vz      [m/s]
Index 4: theta   [rad]
Index 5: omega   [rad/s]
```

`HopperState2D.to_vetor()` konvertiert den semantischen Zustandsdatentyp in diese numerische Darstellung.

`HopperStatte2D.from_vector()` führt die inverse Konvertierung durch.

Die Vektordarstellung ist ausschließlich eine Implementierungsgrenze für numerische Algorithmen. Rohe NumPy-Vektoren werden nicht als öffentliche oder dauerhafte Zustandsdarstellung des Simulators verwendet.

Damit bleibt die fachliche Bedeutung der Zustandsgrößen an den Schnittstellen erhalten, während numerische Algorithmen intern effizient mit Vektoren arbeiten können.
