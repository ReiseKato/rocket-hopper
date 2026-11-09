# Softwarearchitektur

Die Architektur trennt fachliche Konzepte voneinander, ohne eine tiefe Vererbungshierarchie aufzubauen.

## Aktuelle Hauptkomponenten

Die aktuelle Architektur umfasst mehrere klar getrennte Ebenen:

```text
Reference / Guidance
        ↓
Control Layer
        ↓
HopperActuationCommand2D
        ↓
HopperSystem2D
        ↓
HopperActuatorSystem2D
        ↓
HopperActuation2D
        ↓
HopperSimulator
        ↓
Hopper2DDynamics
        ↓
Integrator
```

Die Control Layer kann dabei selbst aus mehreren kaskadierten oder parallelen Reglern bestehen.

Dabei bleiben Regelung, physisches System, Starrkörperdynamik und numerische Integration getrennte Verantwortlichkeiten.

## Starrkörpermodell

```text
HopperState2D
        ↓
HopperActuation2D ──→ Hopper2DDynamics ──→ HopperStateDerivative2D
                              ↑
                   RigidBody2DParameters
```

### `HopperState2D`

Beschreibt ausschließlich den mechanischen Zustand des 2D-Hoppers.

Der Zustand enthält:

\[
[p_x,p_z,v_x,v_z,\theta,\omega]^T.
\]

Aktuatorzustände sind ausdrücklich nicht Bestandteil von `HopperState2D`.

### `RigidBody2DParameters`

Enthält konstante Parameter des Starrkörpermodells:

- Masse,
- Trägheitsmoment um die \(y\)-Achse,
- Engine-Offset,
- Gravitation.

### `HopperActuation2D`

Beschreibt die tatsächlich auf den Starrkörper wirkende physikalische Actuation:

- Schub,
- physikalischer Gimbalwinkel.

Sie enthält keine angeforderten Commands.

### `HopperStateDerivative2D`

Beschreibt die durch die momentane Physik resultierende Zustandsableitung.

### `Hopper2DDynamics`

Implementiert ausschließlich:

\[
(x,u,p)\rightarrow\dot x.
\]

Die Klasse besitzt keinen veränderlichen Simulationszustand.

Sie kennt insbesondere keine:

- Commands,
- Aktuatorgrenzen,
- Controller,
- Integrationsschrittweite.

## Command und physikalische Actuation

Der Fast Simulator trennt ausdrücklich zwischen angeforderter und tatsächlich wirkender Actuation.

`HopperActuationCommand2D` beschreibt:

\[
u_{\mathrm{cmd}}
=
\begin{bmatrix}
T_{\mathrm{cmd}} \\
\delta_{\mathrm{cmd}}
\end{bmatrix}.
\]

`HopperActuation2D` beschreibt dagegen:

\[
u_{\mathrm{physical}}
=
\begin{bmatrix}
T \\
\delta
\end{bmatrix}.
\]

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

Damit bleibt die Starrkörperdynamik unabhängig davon, wie eine physikalische Actuation erzeugt wurde.

## Aktuatormodelle

Die Aktuatormodelle sind Bestandteil des simulierten physischen Systems.

Sie werden in `HopperActuatorSystem2D` zusammengeführt.

Aktuell werden modelliert:

- ein begrenzter Schubaktuator,
- ein winkel- und ratenbegrenzter Gimbalaktuator.

Der Schubaktuator ist in der ersten Version zustandslos.

Der Gimbalaktuator ist zustandsbehaftet, da sein aktueller physikalischer Winkel für die Ratenbegrenzung benötigt wird.

Aktuatorzustände verbleiben bei den jeweiligen Komponenten.

Sie werden nicht in `HopperState2D` integriert.

## Physische Systemschicht

`HopperSystem2D` bildet die Plant-Grenze des aktuell simulierten Hoppers.

Die Klasse kombiniert:

- `HopperActuatorSystem2D`,
- `HopperSimulator`.

Der Ablauf eines Systemschritts lautet:

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

Konzeptionell:

\[
(u_{\mathrm{cmd},k},\Delta t)
\rightarrow
u_{\mathrm{physical},k}
\rightarrow
x_{k+1}.
\]

`HopperSystem2D` enthält keine eigenen Bewegungsgleichungen.

Die Systemschicht orchestriert ausschließlich die physikalischen Teilkomponenten.

### Simulationszeit

Die Simulationszeit wird weiterhin ausschließlich vom `HopperSimulator` verwaltet.

`HopperSystem2D` besitzt keine zweite unabhängige Uhr.

Damit existiert nur eine Quelle der Wahrheit für die Simulationszeit.

### Starrkörperzustand

`HopperSystem2D` stellt den mechanischen Zustand ausdrücklich als `rigid_body_state` bereit.

Die generische Bezeichnung `state` wird vermieden, weil der Starrkörperzustand nach Einführung zustandsbehafteter Aktuatoren nicht mehr den vollständigen Zustand des gesamten simulierten Systems darstellt.

## Semantische und numerische Zustandsdarstellung

Der Simulator unterscheidet bewusst zwischen einer semantischen und einer numerischen Repräsentation des Fahrzeugzustands.

### Semantische Repräsentation

`HopperState2D` ist die kanonische Zustandsdarstellung an den fachlichen Schnittstellen der Starrkörpersimulation.

Die Klasse stellt benannte physikalische Größen bereit, beispielsweise:

```python
state.px
state.vz
state.theta
state.omega
```

Dadurch bleibt die Bedeutung der einzelnen Werte im Code explizit und nachvollziehbar.

Ein Ausdruck wie:

```python
state.theta
```

ist eindeutig als Fahrzeugwinkel erkennbar, während ein Zugriff auf:

```python
state_vector[4]
```

ohne zusätzlichen Kontext nicht selbsterklärend wäre.

### Numerische Repräsentation

Numerische Algorithmen wie Euler- oder Runge-Kutta-Integratoren benötigen Vektoroperationen.

Daher dürfen solche Komponenten den semantischen Zustand intern in einen NumPy-Vektor konvertieren.

Die expliziten Konvertierungsschnittstellen sind:

- `HopperState2D.to_vector()`
- `HopperState2D.from_vector()`
- `HopperStateDerivative2D.to_vector()`

Die numerische Darstellung wird ausschließlich innerhalb numerischer Algorithmen verwendet.

Rohe NumPy-Vektoren sollen nicht als dauerhafter Simulationszustand zwischen fachlichen Komponenten weitergereicht werden.

Die grundlegende Architekturregel lautet daher:

> NumPy-Vektoren sind eine numerische Repräsentation, nicht das fachliche Zustandsmodell.

## Numerische Integration als austauschbare Komponente

Numerische Integration ist eine eigenständige Komponente des Fast Simulators.

Die Hopper-Dynamik beschreibt ausschließlich die kontinuierliche Physik:

\[
\dot{x}=f(x,u,p).
\]

Der Integrator ist dagegen ausschließlich dafür verantwortlich, aus einer numerischen Ableitungsfunktion und einem Zeitschritt einen neuen numerischen Zustand zu bestimmen.

Die beiden Verantwortlichkeiten werden nicht vermischt.

```text
Hopper Dynamics
      │
      │ liefert Zustandsableitung
      ▼
numerische Ableitungsfunktion
      │
      ▼
Integrator
      │
      ▼
neuer numerischer Zustand
```

Integratoren dürfen keine Hopper-spezifische Physik enthalten.

Umgekehrt enthält `Hopper2DDynamics` keine Integrationslogik und kennt insbesondere keine Integrationsschrittweite `dt`.

Dadurch können Integratoren unabhängig entwickelt und ausgetauscht werden.

Aktuell stehen unter anderem zur Verfügung:

```python
EulerIntegrator()
```

und:

```python
RK4Integrator()
```

Ein Wechsel des Integrators darf keine Änderung am Dynamikmodell, an Aktuatormodellen oder an Controllern erfordern.

## Simulator-Schicht

Die Simulator-Schicht verbindet das kontinuierliche Hopper-Dynamikmodell mit einem austauschbaren numerischen Integrator.

Die zentrale Klasse dieser Schicht ist `HopperSimulator`.

Sie enthält keine eigenen Bewegungsgleichungen und implementiert kein numerisches Integrationsverfahren.

### Verantwortlichkeiten

`HopperSimulator` verwaltet:

- den aktuellen `HopperState2D`,
- die aktuelle Simulationszeit,
- das verwendete `Hopper2DDynamics`,
- den verwendeten `Integrator`.

Ein Simulationsschritt erhält eine bereits physikalisch vorliegende `HopperActuation2D` und eine Integrationsschrittweite `dt`.

Konzeptionell gilt:

\[
(x_k,t_k,u_{\mathrm{physical},k},\Delta t)
\longrightarrow
(x_{k+1},t_{k+1}).
\]

Dabei ist:

\[
t_{k+1}=t_k+\Delta t.
\]

### Datenfluss

Die fachliche Hopper-Dynamik arbeitet mit semantischen Modellobjekten, während der Integrator numerische Zustandsvektoren verwendet.

`HopperSimulator` bildet die Grenze zwischen diesen beiden Darstellungen:

```text
HopperState2D
      │
      │ to_vector()
      ▼
numerischer Zustandsvektor
      │
      ▼
Integrator
      │
      │ wertet f(t, x) aus
      ▼
Adapter im HopperSimulator
      │
      ├── HopperState2D.from_vector()
      │
      ├── Hopper2DDynamics.derivative()
      │
      └── HopperStateDerivative2D.to_vector()
      │
      ▼
numerischer Folgezustand
      │
      │ HopperState2D.from_vector()
      ▼
HopperState2D
```

### Zustandsbehaftete und zustandslose Komponenten

`Hopper2DDynamics` und die numerischen Integratoren sind zustandslos.

`HopperSimulator` ist bewusst zustandsbehaftet.

Er repräsentiert einen laufenden Starrkörper-Simulationsprozess und speichert:

\[
x_k
\]

sowie:

\[
t_k.
\]

Der Gimbalaktuator besitzt ebenfalls einen eigenen Zustand, dieser verbleibt jedoch im Aktuatormodell.

Damit ist der vollständige Zustand des simulierten Systems bewusst auf mehrere physikalisch zuständige Komponenten verteilt.

### Nicht enthaltene Verantwortlichkeiten

`HopperSimulator` enthält bewusst keine:

- Controllerlogik,
- Aktuatordynamik,
- Aktuatorbegrenzungen,
- Sensoren,
- Zustandsschätzung,
- Logging- oder Plotlogik,
- Landedetektion,
- Experimentverwaltung.

## Mehrschritt-Simulation

`HopperSimulator` führt weiterhin genau einen Starrkörper-Integrationsschritt aus.

Ein vollständiger Systemschritt wird dagegen durch `HopperSystem2D` koordiniert.

Für Simulationsläufe über mehrere Systemschritte wird `SimulationRunner` verwendet.

Der Runner arbeitet nicht direkt mit `HopperSimulator`.

Der aktuelle Datenfluss lautet:

```text
CommandProvider
       ↓
SimulationRunner
       ↓
HopperSystem2D
       ↓
SimulationResult
```

Der `CommandProvider` erzeugt vor jedem Schritt einen `HopperActuationCommand2D`.

Er darf dazu Controller auswerten und deren Ausgänge beziehungsweise Referenzen kombinieren.

`HopperSystem2D` erzeugt aus dem fertigen Command die tatsächlich wirkende physikalische Actuation und führt den Starrkörperschritt aus.

`SimulationResult` zeichnet anschließend auf:

- Zeitpunkte,
- Starrkörperzustände,
- Commands,
- physikalische Actuations.

Die detaillierte Semantik ist unter [Simulationsläufe](simulation_runs.md) dokumentiert.

## Regelungsschicht

Controller liegen bewusst außerhalb von `HopperSystem2D`.

Damit bleibt die Trennung zwischen Regler und Plant erhalten.

### Direkte Command-Regler

Der Attitude Controller erzeugt:

\[
\delta_{\mathrm{cmd}}.
\]

Der Altitude Controller erzeugt:

\[
T_{\mathrm{cmd}}.
\]

Beide Command-Anteile werden zu einem `HopperActuationCommand2D` kombiniert.

### Äußerer Horizontal Position Controller

Der Horizontal Position Controller erzeugt keinen direkten Plant-Command.

Er erzeugt eine Referenz für den inneren Attitude Controller:

\[
(p_x,v_x)
\rightarrow
\theta_{\mathrm{ref}}.
\]

Die aktuelle Regelungsstruktur lautet:

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
```

Damit ist der Attitude Controller der innere Regelkreis der horizontalen Positionskaskade.

Der Altitude Controller wirkt parallel auf den Schubkanal.

### Physikalische Kopplung

Die Softwarearchitektur trennt die Regler, obwohl die Plant physikalisch gekoppelt bleibt.

Es gilt:

\[
\dot v_x
=
\frac{T}{m}
\sin(\theta+\delta)
\]

und:

\[
\dot v_z
=
\frac{T}{m}
\cos(\theta+\delta)-g.
\]

Eine horizontale Positionskorrektur über eine geneigte Fahrzeuglage beeinflusst daher gleichzeitig die vertikale Dynamik.

Umgekehrt verändert eine Schubkorrektur des Altitude Controllers auch die horizontale Beschleunigungsautorität.

Diese Kopplung wird aktuell nicht durch eine gemeinsame Control Allocation kompensiert.

### Referenzbegrenzung des äußeren Regelkreises

Der Horizontal Position Controller begrenzt die von ihm erzeugte Fahrzeuglagereferenz auf:

\[
|\theta_{\mathrm{ref}}|
\leq
\theta_{\max}.
\]

Diese Begrenzung gehört zur Regelungsanforderung des äußeren Loops und ist von der mechanischen Gimbalbegrenzung zu unterscheiden.

Die physikalischen Gimbalgrenzen verbleiben in `HopperActuatorSystem2D`.

Die Details der Regelungsarchitektur sind unter [Regelung](control/index.md) dokumentiert.

## Visualisierung

Die Visualisierung ist eine nachgelagerte Auswertungsschicht.

Sie arbeitet ausschließlich auf einem `SimulationResult` und ist damit von Dynamics, Integrator, Simulator, Systemschicht und Runner entkoppelt.

```text
SimulationRunner
        ↓
SimulationResult
        ↓
Visualisierung
        ↓
Plots
```

Für die aktuelle Ausbaustufe werden Zeitverläufe von Position, Geschwindigkeit, Lage und physikalischer Actuation dargestellt.

Zusätzlich können Commands und tatsächlich wirkende Actuations miteinander verglichen werden.

Die detaillierte Semantik ist unter [Visualisierung](visualization.md) dokumentiert.

## Architekturregeln

1. Die Starrkörperdynamik begrenzt keine Aktuatoren.
2. Controller-Commands und physikalische Actuation sind getrennte Ebenen.
3. Aktuatormodelle sind Bestandteil der physischen Systemschicht.
4. Controller liegen außerhalb der Plant.
5. Ein äußerer Controller darf Referenzen für einen inneren Controller erzeugen.
6. Ein Controller übernimmt nur die fachlich ihm zugeordnete Regelungsaufgabe.
7. `HopperState2D` beschreibt ausschließlich den Starrkörperzustand.
8. Aktuatorzustände verbleiben bei den jeweiligen Aktuatormodellen.
9. Die Dynamik enthält kein Logging und keine Visualisierung.
10. Numerische Integration ist eine separate Komponente.
11. Der `SimulationRunner` kennt keine konkreten Aktuatormodelle.
12. Gleiche Inputs und Parameter erzeugen deterministisch dieselbe Zustandsableitung.
13. Abstraktionen werden erst eingeführt, wenn ein konkreter Konsument sie benötigt.

## Aktuelle Regelungspipeline

```text
Horizontal Position Reference
        ↓
Horizontal Position Controller
        ↓
Attitude Reference
        ↓
Attitude Controller ───────────────┐
        ↓                          │
     delta_cmd                     │
                                   │
Altitude Reference                 │
        ↓                          │
Altitude Controller               │
        ↓                          │
     thrust_cmd                    │
        │                          │
        └──────────────┬───────────┘
                       ↓
          HopperActuationCommand2D
                       ↓
                HopperSystem2D
                       ↓
                Actuator Models
                       ↓
              HopperActuation2D
                       ↓
               HopperSimulator
                       ↓
               Hopper2DDynamics
                       ↓
              State Derivative
                       ↓
                  Integrator
                       ↓
             Next Rigid-Body State
```

## Langfristige Pipeline

Spätere Guidance- oder Trajektorienkomponenten können Referenzen für die bestehenden Regelkreise erzeugen, ohne die Plant-Schnittstelle zu verändern:

```text
Guidance / Trajectory
        ↓
Control Layer
        ↓
HopperActuationCommand2D
        ↓
HopperSystem2D
        ↓
Actuator Models
        ↓
HopperActuation2D
        ↓
HopperSimulator
        ↓
Hopper2DDynamics
        ↓
Integrator
        ↓
Next Rigid-Body State
```
