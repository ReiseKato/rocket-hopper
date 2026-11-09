# Lageregelung

## Zweck

Die Lageregelung erzeugt den Gimbal-Command für das simulierte physische System.

Sie liegt bewusst außerhalb von `HopperSystem2D`.

Damit bleibt die Grenze zwischen Regler und Plant eindeutig:

```text
AttitudeReference2D
        +
HopperState2D
        │
        ▼
AttitudeController2D
        │
        ▼
delta_cmd
        │
        ▼
HopperActuationCommand2D
        │
        ▼
HopperSystem2D
```

`HopperSystem2D` beschreibt das zu regelnde physische System.

Die Regelung ist nicht Bestandteil der Plant und besitzt keine direkte Kontrolle über Starrkörperdynamik oder Aktuatorzustände.

## Regelungsaufgabe

Die Lageregelung regelt die Orientierung des 2D-Hoppers um die \(y\)-Achse.

Geregelt werden:

\[
\theta
\]

und:

\[
\omega.
\]

Die Referenz wird durch `AttitudeReference2D` beschrieben:

\[
\theta_{\mathrm{ref}}
\]

und:

\[
\omega_{\mathrm{ref}}.
\]

Für eine aufrechte, nicht rotierende Soll-Lage gilt:

\[
\theta_{\mathrm{ref}}=0
\]

und:

\[
\omega_{\mathrm{ref}}=0.
\]

Die Lageregelung übernimmt ausdrücklich keine Höhen- oder Positionsregelung.

Sie kann jedoch eine Referenz verfolgen, die von einem übergeordneten Regler dynamisch erzeugt wurde.

## Rolle als innerer Regelkreis

Mit Einführung der horizontalen Positionsregelung ist der Attitude Controller nicht mehr nur ein eigenständiger Regler mit statischer Soll-Lage.

Er bildet zusätzlich den inneren Regelkreis einer Kaskade:

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
```

Der Horizontal Position Controller entscheidet dabei, welche Fahrzeuglage für die gewünschte horizontale Bewegung benötigt wird.

Der Attitude Controller entscheidet weiterhin ausschließlich, welcher Gimbal-Command benötigt wird, um diese Lage zu erreichen.

Damit bleiben zwei unterschiedliche Aufgaben getrennt:

```text
äußerer Regelkreis:
px, vx
    ↓
theta_ref

innerer Regelkreis:
theta, omega
    ↓
delta_cmd
```

Der Attitude Controller kennt den Horizontal Position Controller nicht.

Für ihn ist jede `AttitudeReference2D` gleichwertig, unabhängig davon, ob sie:

- direkt vorgegeben wurde,
- oder von einem äußeren Positionsregler erzeugt wurde.

## Controller-Schnittstelle

`AttitudeController2D` definiert die gemeinsame Schnittstelle für 2D-Lageregler.

Ein Regler erhält:

- den aktuellen `HopperState2D`,
- eine `AttitudeReference2D`.

Er gibt ausschließlich einen angeforderten Gimbalwinkel zurück:

\[
\delta_{\mathrm{cmd}}.
\]

Konzeptionell:

\[
(x_{\mathrm{RB}},r_{\mathrm{attitude}})
\rightarrow
\delta_{\mathrm{cmd}}.
\]

Der Lageregler erzeugt bewusst keinen vollständigen `HopperActuationCommand2D`.

Insbesondere ist der Schub keine Verantwortung des Attitude Controllers.

Ein `CommandProvider` kombiniert den Gimbal-Command mit dem Schub-Command, der beispielsweise vom Altitude Controller erzeugt wird:

```text
AttitudeController2D
        ↓
delta_cmd
        │
        │
        ├───────────────┐
                        │
AltitudeController2D    │
        ↓               │
thrust_cmd              │
        │               │
        └───────┬───────┘
                ▼
HopperActuationCommand2D
```

Diese Trennung hält Lage-, Höhen- und Positionsregelung unabhängig voneinander testbar.

## PD-Regler

Die erste Implementierung ist `PDAttitudeController2D`.

Der Regler verwendet den Lagefehler

\[
e_\theta
=
\operatorname{wrapToPi}
\left(
\theta-\theta_{\mathrm{ref}}
\right)
\]

und den Winkelgeschwindigkeitsfehler

\[
e_\omega
=
\omega-\omega_{\mathrm{ref}}.
\]

Der Gimbal-Command lautet:

\[
\delta_{\mathrm{cmd}}
=
K_Pe_\theta
+
K_De_\omega.
\]

Die Parameter werden durch `PDAttitudeControllerParameters` beschrieben.

## Vorzeichenkonvention

Das positive Vorzeichen der Reglergleichung ist eine direkte Folge der im Hopper-Modell definierten Gimbalkonvention.

Für den aktuellen 2D-Hopper gilt:

\[
\delta>0
\quad\Rightarrow\quad
\dot\omega<0.
\]

Ein positiver Lagefehler

\[
e_\theta>0
\]

soll eine negative Winkelbeschleunigung erzeugen.

Daher muss der Regler einen positiven Gimbal-Command anfordern:

\[
e_\theta>0
\Rightarrow
\delta_{\mathrm{cmd}}>0
\Rightarrow
\dot\omega<0.
\]

Dasselbe gilt für die Dämpfung einer positiven Winkelgeschwindigkeit:

\[
e_\omega>0
\Rightarrow
\delta_{\mathrm{cmd}}>0
\Rightarrow
\dot\omega<0.
\]

Die Vorzeichenkonvention ist daher Bestandteil der projektspezifischen Reglerdefinition und wird durch Tests abgesichert.

## Winkel-Wrapping

`HopperState2D.theta` bleibt weiterhin ein kontinuierlicher, nicht gewrappter Zustand.

Beispielsweise kann der Starrkörperzustand nach mehreren Umdrehungen enthalten:

\[
\theta=721^\circ.
\]

Für die Regelung wird jedoch nicht der rohe Winkelunterschied verwendet.

Der Lagefehler wird auf das Intervall

\[
[-\pi,\pi)
\]

gewrappt.

Damit wird immer die kürzere Rotationsrichtung als Regelfehler verwendet.

Beispielsweise entsprechen:

\[
\theta=179^\circ
\]

und:

\[
\theta_{\mathrm{ref}}=-179^\circ
\]

nicht einem Regelfehler von \(358^\circ\), sondern:

\[
e_\theta=-2^\circ.
\]

Diese Trennung ist bewusst:

```text
Dynamikzustand:
theta kontinuierlich und ungewrappt

Regelfehler:
auf [-pi, pi) gewrappt
```

## Keine Aktuatorbegrenzungen im Controller

`PDAttitudeController2D` begrenzt seinen Gimbal-Command nicht auf den mechanisch erreichbaren Gimbalwinkel.

Er berücksichtigt ebenfalls keine Gimbalrate.

Ein Controller darf daher beispielsweise anfordern:

\[
\delta_{\mathrm{cmd}}=15^\circ,
\]

auch wenn das physische Gimbalmodell nur

\[
\delta_{\max}=10^\circ
\]

erreichen kann.

Die Begrenzung erfolgt ausschließlich in den Aktuatormodellen von `HopperSystem2D`.

Der Datenfluss bleibt:

```text
Attitude Controller
        ↓
unbegrenzter delta_cmd
        ↓
HopperSystem2D
        ↓
Gimbal-Aktuatormodell
        ↓
physikalisch erreichbarer Gimbalwinkel
```

Damit kennt der Controller die konkrete Aktuatorimplementation nicht.

Diese Trennung ist insbesondere für Untersuchungen von Sättigung und Hardwareanforderungen wichtig.

Die Begrenzung der vom äußeren Horizontal Position Controller erzeugten Fahrzeuglage ist davon zu unterscheiden.

`max_attitude_angle` begrenzt eine Regelungsreferenz:

\[
\theta_{\mathrm{ref}}.
\]

Sie ist keine mechanische Gimbalgrenze.

## Zustandsbehaftung

Der aktuelle PD-Attitude-Controller ist zustandslos.

Sein Ausgang hängt ausschließlich von:

- aktuellem Starrkörperzustand,
- aktueller Referenz,
- festen Reglerparametern

ab.

Es existiert insbesondere kein Integralzustand.

Ein Integralanteil wird in der ersten Version bewusst nicht verwendet.

Damit entstehen aktuell weder Integral-Windup noch zusätzliche Controllerzustände.

## Auslegung der PD-Gains

Die Reglergains können direkt als

\[
K_P
\]

und:

\[
K_D
\]

angegeben werden.

Zusätzlich stellt der Fast Simulator die Hilfsfunktion

```python
design_pd_attitude_gains(...)
```

bereit.

Sie dient dazu, die Gains für einen nominalen Arbeitspunkt aus einer gewünschten Eigenkreisfrequenz und Dämpfung abzuleiten.

### Lokale Rotationsdynamik

Für kleine Gimbalwinkel gilt näherungsweise:

\[
\ddot\theta
\approx
-\frac{lT}{I_y}\delta.
\]

Mit:

\[
\delta_{\mathrm{cmd}}
=
K_P\theta
+
K_D\dot\theta
\]

für eine Nullreferenz ergibt sich lokal:

\[
\ddot\theta
+
\frac{lT}{I_y}K_D\dot\theta
+
\frac{lT}{I_y}K_P\theta
=
0.
\]

Für ein gewünschtes System zweiter Ordnung

\[
\ddot\theta
+
2\zeta\omega_n\dot\theta
+
\omega_n^2\theta
=
0
\]

folgen die Gain-Beziehungen:

\[
K_P
=
\frac{I_y\omega_n^2}{lT_{\mathrm{nom}}}
\]

und:

\[
K_D
=
\frac{2\zeta\omega_nI_y}{lT_{\mathrm{nom}}}.
\]

Dabei bezeichnet:

- \(T_{\mathrm{nom}}\) den nominalen Schub des Auslegungspunkts,
- \(\omega_n\) die gewünschte Eigenkreisfrequenz,
- \(\zeta\) die gewünschte Dämpfung.

Die Hilfsfunktion implementiert ausschließlich diese projektspezifische Zuordnung zum aktuellen kleinen 2D-Rotationsmodell.

## Nominaler Arbeitspunkt

Die Reglerauslegung gilt nur lokal um den angegebenen nominalen Schub.

Die Rotationsautorität des Gimbals hängt direkt von \(T\) ab:

\[
\frac{lT}{I_y}.
\]

Bei größerem Schub erzeugt derselbe Gimbalwinkel ein größeres Drehmoment.

Bei kleinerem Schub sinkt die Regelautorität.

Für:

\[
T=0
\]

kann der Gimbal im aktuellen Modell überhaupt kein Drehmoment erzeugen.

Die erste Reglerauslegung verwendet daher einen ungefähr konstanten Hover-Schub als nominalen Arbeitspunkt.

Eine spätere Regelungsarchitektur kann beispielsweise Gain Scheduling oder eine drehmomentbasierte Control-Allocation-Schicht verwenden, falls die Schubvariation dies erforderlich macht.

## Bandbreite innerhalb der Kaskade

Der Attitude Controller ist im aktuellen kaskadierten Aufbau der schnelle innere Regelkreis.

Der Horizontal Position Controller ist ein langsamerer äußerer Regelkreis.

Daher wird gefordert:

\[
\omega_{n,x}
<
\omega_{n,\theta}.
\]

Im ersten Positionsregelungsversuch werden beispielsweise verwendet:

\[
\omega_{n,x}=0.8\,\mathrm{rad/s}
\]

und:

\[
\omega_{n,\theta}=4\,\mathrm{rad/s}.
\]

Diese Bandbreitentrennung soll sicherstellen, dass der innere Lagekreis die vom äußeren Regler angeforderte Lage ausreichend schnell nachführen kann.

## Einheiten

Die Regelung arbeitet intern mit Radiant.

| Größe | Einheit |
|---|---|
| \(\theta\) | rad |
| \(\omega\) | rad/s |
| \(\delta_{\mathrm{cmd}}\) | rad |
| \(K_P\) | dimensionslos bezüglich rad/rad |
| \(K_D\) | s |
| \(\omega_n\) | rad/s |
| \(\zeta\) | dimensionslos |

Gradangaben werden nur für menschenlesbare Konfigurationen, Ausgaben oder Visualisierungen verwendet.

## Eigenständiger Closed-Loop-Versuch

Das Beispiel `attitude_stabilization.py` testet den Attitude Controller weiterhin isoliert von übergeordneten Positionsreglern.

Es verwendet:

\[
\theta_0=5^\circ,
\]

\[
\omega_0=0,
\]

\[
\theta_{\mathrm{ref}}=0,
\]

\[
\omega_{\mathrm{ref}}=0
\]

und einen konstanten Schub von:

\[
T_{\mathrm{cmd}}=mg.
\]

Für die Beispielauslegung werden verwendet:

\[
\omega_n=4\,\mathrm{rad/s}
\]

und:

\[
\zeta=0.8.
\]

Mit den aktuellen Beispielparametern

\[
I_y=0.08\,\mathrm{kg\,m^2},
\]

\[
l=0.25\,\mathrm m
\]

und:

\[
T_{\mathrm{nom}}=19.62\,\mathrm N
\]

ergeben sich ungefähr:

\[
K_P\approx0.261
\]

und:

\[
K_D\approx0.104\,\mathrm s.
\]

Zusätzlich wird derselbe Attitude Controller im Beispiel `position_stabilization.py` als innerer Regelkreis mit einer dynamisch erzeugten `AttitudeReference2D` verwendet.

## Grenzen der aktuellen Version

Die Lageregelung berücksichtigt bewusst noch keine:

- Integralregelung,
- Störgrößenbeobachtung,
- Gain-Scheduling,
- Control Allocation,
- Sensorfehler,
- Zustandsbeobachter,
- Messrauschen,
- Controllerverzögerungen.

Höhen- und horizontale Positionsregelung existieren inzwischen als getrennte übergeordnete beziehungsweise parallele Regelungskomponenten und werden nicht in den Attitude Controller integriert.
