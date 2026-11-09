# Regelung

## Zweck

Die Regelungsschicht erzeugt Commands und innere Sollwerte, mit denen das simulierte physische Hopper-System beeinflusst wird.

Sie liegt bewusst außerhalb von `HopperSystem2D`.

Damit bleibt die Trennung zwischen Controller und Plant erhalten:

```text
Reference / Guidance
        +
aktueller Starrkörperzustand
        │
        ▼
Controller
        │
        ▼
Command oder innere Referenz
        │
        ▼
weitere Controller / HopperSystem2D
```

`HopperSystem2D` beschreibt das zu regelnde physische System.

Die Controller greifen nicht direkt auf Starrkörperdynamik oder interne Aktuatorzustände zu.

## Architekturprinzip

Die Regelung ist in fachlich getrennte Regelkreise aufgeteilt.

Ein Controller kann entweder:

- einen Command-Anteil für die Plant erzeugen,
- oder eine Referenz für einen inneren Regelkreis erzeugen.

Die physische Systemschicht entscheidet anschließend, welche Actuation tatsächlich erreicht wird.

Für Commands gilt im allgemeinen Fall:

\[
u_{\mathrm{cmd}}
\neq
u_{\mathrm{physical}}.
\]

Die aktuelle Gesamtstruktur lautet:

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
        │
        ├────────────────────┐
                             │
AltitudeReference2D          │
        +                    │
HopperState2D                │
        │                    │
        ▼                    │
AltitudeController2D         │
        │                    │
        ▼                    │
thrust_cmd                   │
        │                    │
        └──────────┬─────────┘
                   ▼
      HopperActuationCommand2D
                   │
                   ▼
             HopperSystem2D
                   │
                   ▼
          physikalische Plant
```

Aktuatorgrenzen und Aktuatordynamik gehören nicht in die Controller.

Dadurch können Controller gegen verschiedene Aktuatormodelle getestet werden, ohne ihre interne Regelungslogik ändern zu müssen.

## Aktueller Regelungsumfang

Der Fast Simulator besitzt derzeit drei Regelungsfunktionen:

1. Lageregelung
2. Höhenregelung
3. horizontale Positionsregelung

Alle drei aktuellen Implementierungen verwenden PD-Strukturen.

Die Verantwortlichkeiten unterscheiden sich jedoch.

### Lageregelung

Die Lageregelung verwendet:

\[
\theta
\]

und:

\[
\omega.
\]

Sie erzeugt:

\[
\delta_{\mathrm{cmd}}.
\]

Der Attitude Controller ist ausschließlich für den angeforderten Gimbalwinkel verantwortlich.

Er kann sowohl eine direkt vorgegebene als auch eine von einem äußeren Positionsregler erzeugte `AttitudeReference2D` verfolgen.

Details sind unter [Lageregelung](attitude.md) dokumentiert.

### Höhenregelung

Die Höhenregelung verwendet:

\[
p_z
\]

und:

\[
v_z.
\]

Sie erzeugt:

\[
T_{\mathrm{cmd}}.
\]

Der Altitude Controller enthält zusätzlich einen Hover-Feedforward-Anteil:

\[
T_{\mathrm{hover}}=mg.
\]

Details sind unter [Höhenregelung](altitude.md) dokumentiert.

### Horizontale Positionsregelung

Die horizontale Positionsregelung verwendet:

\[
p_x
\]

und:

\[
v_x.
\]

Sie erzeugt keinen direkten Aktuator-Command.

Stattdessen erzeugt sie:

\[
\theta_{\mathrm{ref}}.
\]

Konzeptionell:

\[
(p_x,v_x)
\rightarrow
\theta_{\mathrm{ref}}
\rightarrow
\delta_{\mathrm{cmd}}.
\]

Damit ist der Horizontal Position Controller ein äußerer Regelkreis und der Attitude Controller sein innerer Regelkreis.

Details sind unter [Horizontale Positionsregelung](horizontal_position.md) dokumentiert.

## Kaskadierte Regelstruktur

Der Hopper besitzt keinen unabhängigen horizontalen Aktuator.

Horizontale Beschleunigung entsteht durch eine geneigte Schubrichtung.

Deshalb wird horizontale Position nicht direkt auf einen Gimbal-Command abgebildet.

Stattdessen gilt:

```text
Horizontaler Positionsfehler
        ↓
gewünschte horizontale Beschleunigung
        ↓
theta_ref
        ↓
Attitude Controller
        ↓
delta_cmd
```

Diese Kaskade trennt:

- translatorische Regelungsentscheidung,
- rotatorische Lageregelung,
- physikalische Aktuatorwirkung.

Der innere Attitude Loop wird deutlich schneller ausgelegt als der äußere Horizontal Position Loop.

## Zusammensetzen vollständiger Commands

Der Horizontal Position Controller erzeugt zunächst eine `AttitudeReference2D`.

Der Attitude Controller erzeugt daraus:

\[
\delta_{\mathrm{cmd}}.
\]

Parallel erzeugt der Altitude Controller:

\[
T_{\mathrm{cmd}}.
\]

Ein `CommandProvider` kombiniert anschließend beide Command-Anteile:

```python
attitude_reference = (
    horizontal_controller.compute_attitude_reference(
        state=state,
        reference=horizontal_reference,
    )
)

gimbal_command = attitude_controller.compute_gimbal_command(
    state=state,
    reference=attitude_reference,
)

thrust_command = altitude_controller.compute_thrust_command(
    state=state,
    reference=altitude_reference,
)

return HopperActuationCommand2D(
    thrust=thrust_command,
    gimbal_angle=gimbal_command,
)
```

Damit bleiben alle drei Regler unabhängig voneinander testbar.

## Physikalische Kopplung

Die Softwarearchitektur trennt die Regelkreise.

Die Plant selbst bleibt jedoch gekoppelt.

Die horizontale Beschleunigung lautet:

\[
\dot v_x
=
\frac{T}{m}
\sin(\theta+\delta).
\]

Die vertikale Beschleunigung lautet:

\[
\dot v_z
=
\frac{T}{m}
\cos(\theta+\delta)-g.
\]

Eine geneigte Fahrzeuglage für horizontale Bewegung verändert daher gleichzeitig die vertikale Schubkomponente.

Umgekehrt verändert zusätzlicher Schub des Altitude Controllers auch die horizontale Beschleunigungsautorität.

Diese Kopplung wird in der aktuellen ersten Regelungsarchitektur bewusst nicht durch eine gemeinsame Control Allocation versteckt.

## Begrenzung der horizontalen Lagereferenz

Der Horizontal Position Controller besitzt eine maximale zulässige Lagereferenz:

\[
\theta_{\max}.
\]

Diese Grenze ist keine Gimbal-Aktuatorgrenze.

Sie begrenzt die Anforderung des äußeren Regelkreises.

Aus:

\[
\theta_{\max}
\]

wird zunächst:

\[
a_{x,\max}
=
g\sin(\theta_{\max})
\]

abgeleitet.

Der Beschleunigungs-Command wird auf diesen Bereich begrenzt und anschließend über:

\[
\theta_{\mathrm{ref}}
=
\arcsin
\left(
\frac{a_{x,\mathrm{limited}}}{g}
\right)
\]

in eine Lagereferenz umgerechnet.

Die mechanischen Gimbalgrenzen bleiben weiterhin ausschließlich Bestandteil des Aktuatormodells.

## Aktuelle Closed-Loop-Strukturen

Es existieren nun drei sinnvolle Teststufen.

### Nur Lageregelung

```text
AttitudeReference
        ↓
Attitude Controller
        ↓
Gimbal
```

### Lage und Höhe

```text
Attitude Controller ──→ delta_cmd
Altitude Controller ──→ thrust_cmd
```

### Horizontale Position, Lage und Höhe

```text
Horizontal Position Controller
        ↓
Attitude Reference
        ↓
Attitude Controller ──→ delta_cmd

Altitude Controller ─────────→ thrust_cmd
```

Die dritte Stufe schließt damit erstmals alle translatorischen Freiheitsgrade des aktuellen 2D-Modells, die für stationäres Position-Halten relevant sind.

## Aktuelle Beispiele

Die Regelungsbeispiele bauen aufeinander auf.

`attitude_stabilization.py` testet die Lageregelung isoliert.

[Attitude- und Altitude-Stabilisierung](../examples/attitude_altitude_stabilization.md) kombiniert Lage- und Höhenregelung.

[Horizontale Positionsstabilisierung](../examples/position_stabilization.md) ergänzt den äußeren Horizontal Position Controller und verwendet den Attitude Controller erstmals als inneren Regelkreis einer Kaskade.

## Geplante Erweiterungen

Die Regelungsdokumentation wird erst erweitert, wenn entsprechende Komponenten tatsächlich implementiert werden.

Mögliche spätere Bereiche sind:

- Trajektorienregelung statt statischer Positionsreferenzen,
- Guidance,
- Control Allocation,
- Gain Scheduling,
- explizite Neigungs- und Schubkompensation,
- Integralanteile und Anti-Windup,
- Zustandsbeobachtung.

Für diese Themen werden bewusst noch keine leeren Dokumentationsseiten angelegt.

## Dokumentationsstruktur

Aktuell besteht die Regelungsdokumentation aus:

```text
control/
├── index.md
├── attitude.md
├── altitude.md
└── horizontal_position.md
```

`index.md` beschreibt die übergeordnete Regelungsarchitektur.

`attitude.md` beschreibt die Lageregelung.

`altitude.md` beschreibt die Höhenregelung.

`horizontal_position.md` beschreibt die äußere horizontale Positionsregelung und ihre Kaskade mit dem Attitude Controller.

Weitere Unterseiten werden nur ergänzt, wenn neue Regelungskomponenten tatsächlich Teil des Fast Simulators werden.
