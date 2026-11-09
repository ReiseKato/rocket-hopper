# Aktuatormodelle

## Zweck

Die Aktuatormodelle bilden die Grenze zwischen angeforderten Steuergrößen und der tatsächlich auf den Starrkörper wirkenden Actuation.

Der Fast Simulator unterscheidet daher zwischen

\[
u_{\mathrm{cmd}}
\]

und

\[
u_{\mathrm{physical}}.
\]

`HopperActuationCommand2D` beschreibt gewünschte Werte.

`HopperActuation2D` beschreibt dagegen die tatsächlich von den Aktuatormodellen erzeugten physikalischen Größen.

Der Datenfluss lautet:

```text
HopperActuationCommand2D
        ↓
Aktuatormodelle
        ↓
HopperActuation2D
        ↓
Hopper2DDynamics
```

Diese Trennung verhindert, dass Controller oder Experimente von idealen, physikalisch nicht erreichbaren Aktuatoren ausgehen.

## Modellumfang der ersten Version

Die erste Aktuatorversion bleibt bewusst einfach.

Sie soll zunächst nur die für das aktuelle Hopper-Modell wichtigsten physikalischen Begrenzungen abbilden, ohne bereits konkrete Hardwaredetails vorwegzunehmen.

### Schubaktuator

Der Schubaktuator berücksichtigt ausschließlich einen minimalen und maximalen erreichbaren Schub:

\[
T_{\min}
\leq
T
\leq
T_{\max}.
\]

Der gewünschte Schub wird entsprechend begrenzt:

\[
T
=
\operatorname{clip}
\left(
T_{\mathrm{cmd}},
T_{\min},
T_{\max}
\right).
\]

Das aktuelle Modell besitzt keine eigene zeitliche Dynamik.

Insbesondere werden noch keine folgenden Effekte modelliert:

- Motorzeitkonstante,
- Drehzahl,
- Schubänderungsrate,
- Batteriespannung,
- ESC-Verhalten,
- elektrische Leistungsaufnahme.

Der Schubaktuator ist in dieser ersten Version daher zustandslos.

### Gimbalaktuator

Der Gimbalaktuator berücksichtigt:

- einen maximalen absoluten Gimbalwinkel,
- eine maximale Gimbalgeschwindigkeit.

Der gewünschte Winkel wird zunächst auf den zulässigen Bereich begrenzt:

\[
\delta_{\mathrm{target}}
=
\operatorname{clip}
\left(
\delta_{\mathrm{cmd}},
-\delta_{\max},
+\delta_{\max}
\right).
\]

Die innerhalb eines Simulationsschritts maximal mögliche Winkeländerung beträgt:

\[
\Delta \delta_{\max}
=
\dot{\delta}_{\max}\Delta t.
\]

Ausgehend vom aktuell erreichten Gimbalwinkel wird die tatsächlich ausgeführte Änderung begrenzt:

\[
\Delta\delta
=
\operatorname{clip}
\left(
\delta_{\mathrm{target}}-\delta_k,
-\Delta\delta_{\max},
+\Delta\delta_{\max}
\right).
\]

Der neue Gimbalwinkel ergibt sich zu:

\[
\delta_{k+1}
=
\delta_k+\Delta\delta.
\]

Damit kann der Gimbal einen angeforderten Winkel nicht instantan erreichen, wenn die dafür notwendige Winkelgeschwindigkeit größer als die konfigurierte maximale Gimbalrate wäre.

## Zustandsbehaftung

Das aktuelle Schubmodell ist zustandslos.

Das Gimbalmodell ist dagegen zustandsbehaftet, da die maximal mögliche Winkeländerung vom bereits erreichten Gimbalwinkel abhängt.

Der aktuelle Gimbalwinkel ist daher Zustand des Gimbalaktuatormodells und nicht Bestandteil von `HopperState2D`.

`HopperState2D` beschreibt weiterhin ausschließlich den Starrkörperzustand:

\[
x_{\mathrm{RB}}
=
\begin{bmatrix}
p_x &
p_z &
v_x &
v_z &
\theta &
\omega
\end{bmatrix}^{T}.
\]

Aktuatorzustände bleiben bei den physikalischen Komponenten, denen sie zugeordnet sind.

Dadurch wird vermieden, den Starrkörperzustand schrittweise um fachlich fremde Zustände zu erweitern.

## Diskrete Aktualisierung des Gimbals

Das aktuelle Gimbalmodell ist ein diskretes Aktuatormodell.

Zu Beginn eines Systemschritts wird aus

- dem angeforderten Gimbalwinkel,
- dem aktuellen Gimbalwinkel,
- der maximalen Gimbalrate,
- der Schrittweite `dt`

der neue physikalische Gimbalwinkel bestimmt.

Dieser Wert wird anschließend während des vollständigen Starrkörper-Integrationsschritts konstant gehalten.

```text
Command bei t_k
      ↓
Gimbal-Aktuatormodell
      ↓
physikalischer Gimbalwinkel
      ↓
konstant über [t_k, t_k+1)
      ↓
Rigid-Body-Integration
```

Damit bleibt innerhalb eines Integrationsschritts die bereits definierte Zero-Order-Hold-Annahme erhalten.

Die interne Auswertung des Integrators, beispielsweise die mehreren Ableitungsauswertungen von RK4, verwendet während desselben Starrkörperschritts dieselbe physikalische Actuation.

Diese Modellierung ist eine bewusste Vereinfachung der ersten Simulatorversion.

Eine kontinuierliche Servo- oder Gimbaldynamik kann später als austauschbares Aktuatormodell ergänzt werden, wenn reale Hardware oder eine konkrete Untersuchungsfrage dies erforderlich macht.

## Command und physikalische Actuation

Der Fast Simulator unterscheidet explizit zwischen gewünschter und tatsächlich erreichter Actuation.

Konzeptionell gilt:

\[
u_{\mathrm{cmd}}
=
\begin{bmatrix}
T_{\mathrm{cmd}} \\
\delta_{\mathrm{cmd}}
\end{bmatrix}
\]

und:

\[
u_{\mathrm{physical}}
=
\begin{bmatrix}
T \\
\delta
\end{bmatrix}.
\]

`HopperActuationCommand2D` repräsentiert den angeforderten Zustand vor Anwendung der Aktuatorgrenzen.

`HopperActuation2D` repräsentiert den tatsächlich auf den Starrkörper wirkenden Zustand nach Anwendung der Aktuatormodelle.

Die Starrkörperdynamik erhält ausschließlich `HopperActuation2D`.

Sie kennt weder Commands noch Aktuatorgrenzen.

## HopperActuatorSystem2D

Die einzelnen Aktuatormodelle werden in `HopperActuatorSystem2D` zusammengeführt.

Diese Komponente koordiniert:

- den Schubaktuator,
- den Gimbalaktuator.

Sie übernimmt die Abbildung

\[
HopperActuationCommand2D
\rightarrow
HopperActuation2D.
\]

Der Datenfluss lautet:

```text
HopperActuationCommand2D
        │
        ├── thrust command
        │       ↓
        │   ThrustActuator
        │
        └── gimbal command
                ↓
            GimbalActuator
                │
                ▼
        HopperActuation2D
```

`HopperActuatorSystem2D` enthält selbst keine Starrkörperdynamik.

Seine Aufgabe ist ausschließlich die Koordination der Aktuatormodelle und die Erzeugung der tatsächlich wirkenden Actuation.

## Austauschbarkeit

Schub- und Gimbalmodelle werden über getrennte Aktuatorschnittstellen beschrieben.

Dadurch können spätere, realistischere Modelle ergänzt werden, ohne die Starrkörperdynamik oder die physische Systemschicht grundlegend zu verändern.

Mögliche spätere Implementierungen sind beispielsweise:

```text
ThrustActuator
    ├── LimitedThrustActuator
    ├── FirstOrderThrustActuator
    └── MeasuredThrustActuator

GimbalActuator
    ├── RateLimitedGimbalActuator
    ├── FirstOrderGimbalActuator
    └── MeasuredServoGimbalActuator
```

Solche Modelle sollen erst ergänzt werden, wenn ein konkreter physikalischer oder experimenteller Bedarf besteht.

## Einheiten

Die Aktuatormodelle verwenden intern dieselben Einheiten wie das restliche Dynamikmodell.

| Größe | Einheit |
|---|---|
| Schub | N |
| Gimbalwinkel | rad |
| Gimbalrate | rad/s |
| Zeit | s |

Gradangaben können für Konfiguration, Ausgabe oder Visualisierung verwendet werden, werden intern jedoch in Radiant umgerechnet.

## Verhältnis zur Systemschicht

Aktuatormodelle sind Bestandteil des simulierten physischen Hoppers und nicht Bestandteil der Ablaufsteuerung eines Experiments.

Sie werden daher nicht im `SimulationRunner` implementiert.

Stattdessen werden sie innerhalb der physischen Systemschicht verwendet:

```text
HopperActuationCommand2D
        ↓
HopperActuatorSystem2D
        ↓
HopperActuation2D
        ↓
HopperSimulator
        ↓
HopperState2D
```

`HopperSystem2D` koordiniert diese Komponenten zu einem vollständigen Systemschritt.

Dadurch bleiben:

- Experimentablauf,
- Aktuatorphysik,
- Starrkörperdynamik

klar voneinander getrennt.

## Nicht modellierte Effekte

In der aktuellen Version werden insbesondere noch nicht berücksichtigt:

- Servozeitkonstanten,
- Totzeiten,
- Backlash,
- Deadband,
- Hysterese,
- mechanische Nachgiebigkeit,
- Schubänderungsraten,
- Motordrehzahl,
- ESC-Dynamik,
- Batterieeffekte,
- Spannungsabfall,
- elektrische Ansteuerung,
- Temperaturabhängigkeiten.

Diese Effekte werden erst ergänzt, wenn reale Hardware oder eine konkrete Fragestellung ihre Modellierung rechtfertigt.

## Ziel der ersten Aktuatorversion

Die erste Aktuatorversion soll vor allem verhindern, dass spätere Controller auf idealisierte, physikalisch nicht erreichbare Actuation zurückgreifen können.

Insbesondere sollen damit Fragen untersucht werden können wie:

- Reicht der maximal verfügbare Schub für die gewünschte Flugphase?
- Ist der verfügbare Gimbalwinkel ausreichend?
- Ist die maximale Gimbalrate hoch genug für eine stabile Lageregelung?
- Wie stark unterscheiden sich Command und tatsächlich wirkende Actuation?

Damit bilden die Aktuatormodelle die erste direkte Verbindung zwischen der abstrakten Simulationsarchitektur und späteren realen Hardwareanforderungen.
