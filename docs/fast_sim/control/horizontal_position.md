# Horizontale Positionsregelung

## Zweck

Die horizontale Positionsregelung schließt den bisher offenen horizontalen Freiheitsgrad des 2D-Hoppers.

Geregelt werden:

\[
p_x
\]

und:

\[
v_x.
\]

Der Controller erzeugt dabei **keinen Gimbal-Command**.

Stattdessen erzeugt er eine gewünschte Fahrzeuglage:

\[
\theta_{\mathrm{ref}}.
\]

Diese Referenz wird anschließend vom bestehenden Attitude Controller verfolgt.

Damit entsteht erstmals eine kaskadierte Regelstruktur:

```text
HorizontalPositionReference2D
        +
HopperState2D
        │
        ▼
HorizontalPositionController2D
        │
        ▼
AttitudeReference2D
        │
        ▼
AttitudeController2D
        │
        ▼
delta_cmd
        │
        ▼
HopperSystem2D
```

## Warum die horizontale Position nicht direkt über den Gimbal geregelt wird

Die horizontale Bewegung hängt im aktuellen Starrkörpermodell von der Richtung und Größe des Schubvektors ab:

\[
\dot v_x
=
\frac{T}{m}
\sin(\theta+\delta).
\]

Der Gimbal beeinflusst diese Bewegung auf zwei Arten:

1. ein von null verschiedener Gimbalwinkel verändert die Schubrichtung unmittelbar,
2. der Gimbal erzeugt ein Drehmoment und verändert dadurch die Fahrzeuglage \(\theta\).

Für eine stationäre geneigte Lage gilt nach dem Ausregeln typischerweise näherungsweise:

\[
\delta\approx0.
\]

Die horizontale Beschleunigung kann dann trotzdem von null verschieden sein, weil:

\[
\theta\neq0.
\]

Der Horizontal Position Controller soll daher nicht selbst bestimmen, welcher Gimbalwinkel nötig ist.

Seine Aufgabe lautet stattdessen:

> Welche Fahrzeuglage wird benötigt, um den horizontalen Positions- und Geschwindigkeitsfehler abzubauen?

Die Umsetzung dieser gewünschten Lage bleibt Aufgabe des inneren Attitude Controllers.

## Referenz

`HorizontalPositionReference2D` beschreibt:

\[
p_{x,\mathrm{ref}}
\]

und:

\[
v_{x,\mathrm{ref}}.
\]

Für das Halten einer horizontalen Position gilt typischerweise:

\[
v_{x,\mathrm{ref}}=0.
\]

## Controller-Schnittstelle

`HorizontalPositionController2D` erhält:

- den aktuellen `HopperState2D`,
- eine `HorizontalPositionReference2D`.

Er gibt eine `AttitudeReference2D` zurück.

Konzeptionell:

\[
(x_{\mathrm{RB}},r_x)
\rightarrow
r_{\mathrm{attitude}}.
\]

Die aktuelle Implementierung erzeugt:

\[
r_{\mathrm{attitude}}
=
\left(
\theta_{\mathrm{ref}},
\omega_{\mathrm{ref}}
\right)
\]

mit:

\[
\omega_{\mathrm{ref}}=0.
\]

Der Attitude Controller erhält damit dieselbe Referenzstruktur unabhängig davon, ob die gewünschte Lage:

- direkt vorgegeben wurde,
- oder von einem äußeren Positionsregler erzeugt wurde.

## PD-Regler

Die erste Implementierung ist `PDHorizontalPositionController2D`.

Der Positionsfehler lautet:

\[
e_x
=
p_{x,\mathrm{ref}}-p_x.
\]

Der Geschwindigkeitsfehler lautet:

\[
e_{v_x}
=
v_{x,\mathrm{ref}}-v_x.
\]

Aus diesen Fehlern wird zunächst eine gewünschte horizontale Beschleunigung erzeugt:

\[
a_{x,\mathrm{cmd}}
=
K_Pe_x
+
K_De_{v_x}.
\]

Diese Zwischenstufe ist bewusst gewählt.

Der äußere Regler entscheidet damit zunächst über die benötigte translatorische Wirkung.

Er bestimmt nicht direkt einen Winkel.

## Zusammenhang zwischen horizontaler Beschleunigung und Fahrzeuglage

Für einen aufrechten beziehungsweise moderat geneigten Hopper mit:

\[
T\approx mg
\]

und einem durch den Attitude Controller näherungsweise ausgeregelten Gimbalwinkel:

\[
\delta\approx0
\]

gilt:

\[
\dot v_x
\approx
g\sin\theta.
\]

Damit ergibt sich für eine gewünschte horizontale Beschleunigung:

\[
\theta_{\mathrm{ref}}
=
\arcsin
\left(
\frac{a_{x,\mathrm{cmd}}}{g}
\right).
\]

Diese Beziehung wird im aktuellen Controller zur Erzeugung der Attitude Reference verwendet.

## Physikalisch konsistente Begrenzung

Eine beliebig große Lagereferenz wäre für den äußeren Positionsregler nicht sinnvoll.

Große Winkel:

- verlassen den vorgesehenen Arbeitspunkt,
- reduzieren die vertikale Schubkomponente stark,
- stellen hohe Anforderungen an den Attitude Controller,
- können die Höhenregelung unnötig belasten.

Daher wird eine maximale absolute Lagereferenz definiert:

\[
\theta_{\max}.
\]

Die Begrenzung wird nicht erst nachträglich auf den Winkel angewendet.

Stattdessen wird aus \(\theta_{\max}\) zunächst die maximal zulässige horizontale Beschleunigungsanforderung abgeleitet:

\[
a_{x,\max}
=
g\sin(\theta_{\max}).
\]

Der rohe Beschleunigungs-Command wird dann begrenzt:

\[
a_{x,\mathrm{limited}}
=
\operatorname{clip}
\left(
a_{x,\mathrm{cmd}},
-a_{x,\max},
+a_{x,\max}
\right).
\]

Erst danach wird die Attitude Reference berechnet:

\[
\boxed{
\theta_{\mathrm{ref}}
=
\arcsin
\left(
\frac{a_{x,\mathrm{limited}}}{g}
\right)
}
\]

Damit ist garantiert:

\[
|\theta_{\mathrm{ref}}|
\leq
\theta_{\max}.
\]

Die Begrenzung bleibt dabei konsistent mit derselben physikalischen Beziehung, die für die Beschleunigungs-zu-Winkel-Abbildung verwendet wird.

## Bedeutung der Begrenzung

`max_attitude_angle` ist keine mechanische Gimbalgrenze.

Sie beschreibt eine Begrenzung der **vom äußeren Regelkreis angeforderten Fahrzeuglage**.

Die Gimbalgrenzen bleiben weiterhin ausschließlich Bestandteil des physischen Aktuatormodells.

Damit existieren zwei getrennte Grenzen:

```text
Horizontal Position Controller
        │
        │ begrenzt theta_ref
        ▼
Attitude Controller
        │
        │ erzeugt delta_cmd
        ▼
Gimbal Actuator
        │
        │ begrenzt delta und delta_dot
        ▼
physikalische Actuation
```

## Vorzeichenkonvention

Liegt die Sollposition rechts vom Hopper:

\[
p_{x,\mathrm{ref}}>p_x,
\]

dann gilt:

\[
e_x>0.
\]

Der Controller erzeugt eine positive Beschleunigungsanforderung:

\[
a_{x,\mathrm{cmd}}>0
\]

und damit:

\[
\theta_{\mathrm{ref}}>0.
\]

Eine positive Fahrzeuglage neigt den Schubvektor in positive \(x\)-Richtung.

Damit entsteht eine positive horizontale Beschleunigung.

Umgekehrt gilt bei einer positiven Driftgeschwindigkeit nach rechts und einer Sollgeschwindigkeit von null:

\[
v_x>0,
\qquad
v_{x,\mathrm{ref}}=0,
\]

also:

\[
e_{v_x}<0.
\]

Der D-Anteil kann dadurch eine negative Lagereferenz erzeugen.

Der Hopper kippt nach links und baut die positive horizontale Geschwindigkeit ab.

## Gain-Auslegung

Die Hilfsfunktion:

```python
design_pd_horizontal_position_gains(...)
```

erzeugt die Gains aus:

- gewünschter Eigenkreisfrequenz \(\omega_n\),
- gewünschter Dämpfung \(\zeta\).

Unter der Annahme, dass der innere Attitude Loop die angeforderte Lage ausreichend schnell nachführt, gilt näherungsweise:

\[
\ddot p_x
=
a_{x,\mathrm{cmd}}.
\]

Mit:

\[
a_{x,\mathrm{cmd}}
=
K_Pe_x
+
K_De_{v_x}
\]

ergibt sich bei konstanter Positionsreferenz:

\[
\ddot e_x
+
K_D\dot e_x
+
K_Pe_x
=
0.
\]

Verglichen mit:

\[
\ddot e_x
+
2\zeta\omega_n\dot e_x
+
\omega_n^2e_x
=
0
\]

folgt:

\[
K_P
=
\omega_n^2
\]

und:

\[
K_D
=
2\zeta\omega_n.
\]

## Bandbreitentrennung

Der Horizontal Position Controller ist ein äußerer Regelkreis.

Der Attitude Controller ist der innere Regelkreis.

Der äußere Regelkreis muss deshalb langsamer ausgelegt werden als der innere.

Konzeptionell:

\[
\omega_{n,x}
<
\omega_{n,\theta}.
\]

Für den ersten Versuch werden verwendet:

\[
\omega_{n,x}=0.8\,\mathrm{rad/s}
\]

und:

\[
\zeta_x=0.9.
\]

Der Attitude Controller bleibt dagegen bei:

\[
\omega_{n,\theta}=4\,\mathrm{rad/s}.
\]

Damit besitzt der innere Lagekreis eine deutlich höhere Bandbreite.

## Beispielparameter

Für:

\[
\omega_{n,x}=0.8\,\mathrm{rad/s}
\]

und:

\[
\zeta_x=0.9
\]

ergeben sich:

\[
K_P=0.64\,\mathrm{s^{-2}}
\]

und:

\[
K_D=1.44\,\mathrm{s^{-1}}.
\]

Für die erste Lagereferenzbegrenzung wird verwendet:

\[
\theta_{\max}=10^\circ.
\]

Bei:

\[
g=9.81\,\mathrm{m/s^2}
\]

folgt:

\[
a_{x,\max}
=
9.81\sin(10^\circ)
\approx
1.70\,\mathrm{m/s^2}.
\]

## Kopplung mit der Höhenregelung

Die horizontale und vertikale Dynamik sind physikalisch gekoppelt.

Wenn der Horizontal Position Controller eine geneigte Fahrzeuglage anfordert, sinkt die vertikale Komponente des Schubvektors.

Es gilt:

\[
T_z
=
T\cos(\theta+\delta).
\]

Die Höhenregelung kann darauf reagieren, indem sie den Schub erhöht.

Damit ergibt sich:

```text
Horizontal Position Controller
        ↓
theta_ref
        ↓
Attitude Controller
        ↓
geneigte Fahrzeuglage
        ↓
veränderte Schubrichtung
        ↓
Altitude Controller reagiert auf pz und vz
```

Die Regler sind softwareseitig getrennt, wirken aber auf dieselbe gekoppelte Plant.

## Abhängigkeit vom Schub

Die verwendete Beziehung:

\[
a_x\approx g\sin\theta
\]

setzt einen Arbeitspunkt mit ungefähr:

\[
T\approx mg
\]

voraus.

Im vollständigen Modell gilt:

\[
\dot v_x
=
\frac{T}{m}
\sin(\theta+\delta).
\]

Ändert der Altitude Controller den Schub deutlich, ändert sich daher auch die tatsächliche horizontale Beschleunigung bei gleicher Fahrzeuglage.

Die erste Version des Horizontal Position Controllers berücksichtigt diese Schubvariation nicht explizit.

Sie bleibt als reale Kopplung der simulierten Plant sichtbar.

Eine spätere Regelungsarchitektur kann bei Bedarf eine schubabhängige Beschleunigungsabbildung oder eine gemeinsame Control-Allocation-Struktur verwenden.

## Verantwortungsgrenzen

`HorizontalPositionController2D` übernimmt bewusst keine:

- Gimbalregelung,
- Schubregelung,
- Höhenregelung,
- Aktuatorbegrenzung,
- Aktuatordynamik,
- direkte Plant-Simulation,
- Trajektorienplanung.

Seine fachliche Aufgabe ist ausschließlich:

\[
(p_x,v_x,\text{Referenz})
\rightarrow
\theta_{\mathrm{ref}}.
\]

## Grenzen der ersten Version

Die erste horizontale Positionsregelung besitzt noch keine:

- Integralregelung,
- Trajektorienreferenzen mit Beschleunigung,
- explizite Schubkompensation,
- Gain-Scheduling,
- Anti-Windup-Mechanismen,
- gemeinsame Control Allocation,
- Beschleunigungs- oder Jerk-Limits,
- Sensorfehler,
- Zustandsschätzung.

Diese Erweiterungen werden erst ergänzt, wenn die einfache kaskadierte Positionsregelung validiert wurde.
