# Visualisierung

## Zweck

Die Visualisierungsschicht dient der Auswertung eines bereits abgeschlossenen Simulationslaufs.

Sie arbeitet ausschließlich auf einem `SimulationResult` und besitzt keine Kenntnis über die internen Details der Simulationserzeugung.

Insbesondere kennt die Visualisierung keine:

- Bewegungsgleichungen,
- Integrationsverfahren,
- Controller,
- konkreten Aktuatormodelle,
- Experimentlogik.

Sie verwendet ausschließlich die im `SimulationResult` gespeicherten Daten.

Die Visualisierung ist damit ein reiner Nachverarbeitungsschritt:

\[
\text{SimulationResult}
\rightarrow
\text{Plots}.
\]

## Grundprinzip

Für die erste Ausbaustufe werden einfache Zeitverlaufsplots erzeugt.

Der aktuelle Standardumfang umfasst:

- \(p_x(t)\)
- \(p_z(t)\)
- \(\theta(t)\)
- \(\omega(t)\)
- \(T(t)\)
- \(\delta(t)\)

Jede dieser Größen wird in einer eigenen Figure dargestellt.

Zusätzlich können die angeforderten Commands direkt mit der tatsächlich wirkenden physikalischen Actuation verglichen werden:

- \(T_{\mathrm{cmd}}(t)\) gegenüber \(T(t)\)
- \(\delta_{\mathrm{cmd}}(t)\) gegenüber \(\delta(t)\)

Die Visualisierung enthält keine eigene Simulationslogik und verändert die gespeicherten Simulationsdaten nicht.

## Daten in SimulationResult

`SimulationResult` enthält für einen Lauf mit \(N\) Systemschritten:

- \(N+1\) Zeitpunkte,
- \(N+1\) Starrkörperzustände,
- \(N\) Commands,
- \(N\) tatsächlich angewandte physikalische Actuations.

Damit gilt:

\[
|\text{times}|
=
|\text{rigid\_body\_states}|
=
N+1
\]

und:

\[
|\text{commands}|
=
|\text{physical\_actuations}|
=
N.
\]

Die Starrkörperzustände gehören direkt zu den gespeicherten Zeitpunkten:

\[
x_{\mathrm{RB},k}
\text{ bei }
t_k.
\]

Commands und physikalische Actuations gehören dagegen jeweils zu einem Systemschritt.

Konzeptionell:

```text
t0              t1              t2
●---------------●---------------●
x0              x1              x2

    command0        command1
       ↓               ↓
    actuation0      actuation1
```

Dabei ist `commands[k]` der zu Beginn des Schritts angeforderte Command.

`physical_actuations[k]` ist die daraus vom physischen System erzeugte Actuation, die für die Starrkörperintegration des Intervalls

\[
[t_k,t_{k+1})
\]

verwendet wird.

Diese Unterscheidung ist für die Visualisierung wesentlich.

## Darstellung von Zuständen

Zustandsgrößen wie Position, Fahrzeugwinkel und Winkelgeschwindigkeit werden als gewöhnliche Linienplots dargestellt.

Sie verwenden die gespeicherten Zeitpunkte `result.times` und die zugehörigen Starrkörperzustände `result.rigid_body_states`.

Konzeptionell:

```text
times[k]   ↔   rigid_body_states[k]
```

Das gilt für:

- `px`
- `pz`
- `theta`
- `omega`

## Darstellung der physikalischen Actuation

Die tatsächlich wirkenden Actuation-Signale werden nicht als gewöhnliche Linienplots dargestellt.

Da jede `HopperActuation2D` während eines Starrkörper-Integrationsschritts konstant gehalten wird, ist eine treppenförmige Darstellung fachlich passend.

Konzeptionell gilt:

```text
physical_actuations[k]   ↔   [times[k], times[k+1])
```

Für die Visualisierung von:

- `thrust`
- `gimbal_angle`

werden daher Treppenplots verwendet.

Die Basisplots zeigen ausdrücklich die **tatsächlich auf den Starrkörper wirkende Actuation** und nicht den ursprünglichen Command.

## Vergleich von Command und physikalischer Actuation

Neben den Basisplots können Command und tatsächlich wirkende Actuation direkt miteinander verglichen werden.

Dafür stehen die Funktionen

```python
plot_thrust_command_comparison(result)
```

und

```python
plot_gimbal_angle_command_comparison(result)
```

zur Verfügung.

Beide Vergleichsplots können gemeinsam mit

```python
plot_actuation_comparisons(result)
```

erzeugt werden.

### Semantik

Für einen Systemschritt \(k\) enthält `SimulationResult` sowohl

\[
u_{\mathrm{cmd},k}
\]

als auch

\[
u_{\mathrm{physical},k}.
\]

Der Command wird am Anfang des Systemschritts bestimmt.

Anschließend erzeugen die Aktuatormodelle innerhalb von `HopperSystem2D` daraus die tatsächlich wirkende Actuation:

\[
u_{\mathrm{cmd},k}
\rightarrow
u_{\mathrm{physical},k}.
\]

Beide Größen werden dem Systemschritt \(k\) beziehungsweise dem Intervall

\[
[t_k,t_{k+1})
\]

zugeordnet und daher als Treppenfunktionen dargestellt.

### Bedeutung der Vergleichsplots

Unterschiede zwischen Command und physikalischer Actuation machen die Wirkung der Aktuatormodelle direkt sichtbar.

Beim Schub können beispielsweise Sättigungsgrenzen sichtbar werden:

\[
T_{\mathrm{cmd}}>T_{\max}
\quad\Rightarrow\quad
T=T_{\max}.
\]

Beim Gimbal können sowohl Winkel- als auch Ratenbegrenzungen sichtbar werden.

Ein rate-limitierter Gimbal kann einem sprunghaften Command daher nur schrittweise folgen.

### Erster physikalischer Wert eines zustandsbehafteten Aktuators

Bei einem zustandsbehafteten Aktuatormodell ist zu beachten, dass `physical_actuations[k]` bereits das Ergebnis der Aktuatoraktualisierung für den Systemschritt \(k\) darstellt.

Startet der Gimbal beispielsweise bei

\[
\delta_0=0^\circ,
\]

darf sich innerhalb eines Schritts um maximal

\[
1^\circ
\]

bewegen und erhält zu Beginn des ersten Schritts einen Command von

\[
5^\circ,
\]

dann gilt für das erste Integrationsintervall bereits:

\[
\delta_{\mathrm{physical},0}=1^\circ.
\]

Der vorherige interne Gimbalzustand von \(0^\circ\) ist nicht selbst Eintrag der `physical_actuations`-Historie.

## Einheiten in der Visualisierung

Intern arbeitet das Modell vollständig in SI-Einheiten und verwendet Winkel in Radiant.

Für die visuelle Darstellung wird davon teilweise bewusst abgewichen, wenn dies die Lesbarkeit verbessert.

### Zustandsgrößen

- `px` in m
- `pz` in m
- `theta` in deg
- `omega` in deg/s

### Commands und Actuation

- `thrust` in N
- `gimbal_angle` in deg

Die Umrechnung von Radiant in Grad ist ausschließlich Teil der Visualisierung.

Die gespeicherten Zustands-, Command- und Actuation-Daten bleiben unverändert.

## Öffentliche Schnittstelle

Die zentrale Einstiegsfunktion für die Basisplots ist:

```python
plot_basic_time_series(result)
```

Sie erzeugt den Standardumfang der sechs Basisplots und liefert diese als Dictionary von Figure-Objekten zurück.

Zusätzlich existieren Funktionen für einzelne Zeitverläufe:

- `plot_px_time_series(...)`
- `plot_pz_time_series(...)`
- `plot_theta_time_series(...)`
- `plot_omega_time_series(...)`
- `plot_thrust_time_series(...)`
- `plot_gimbal_angle_time_series(...)`

Für die Gegenüberstellung von Command und tatsächlich wirkender Actuation existieren:

- `plot_thrust_command_comparison(...)`
- `plot_gimbal_angle_command_comparison(...)`
- `plot_actuation_comparisons(...)`

Dadurch können Basisplots und Actuation-Vergleiche unabhängig voneinander erzeugt werden.

## Verantwortung der Visualisierung

Die Visualisierungsschicht ist nicht dafür verantwortlich:

- Simulationsläufe zu starten,
- Aktuatormodelle auszuführen,
- Ergebnisse zu speichern,
- Dateien automatisch zu schreiben,
- Experimente zu konfigurieren,
- Controllerverhalten zu implementieren.

Sie erzeugt ausschließlich Figure-Objekte auf Basis eines `SimulationResult`.

Ob diese angezeigt, gespeichert oder weiterverarbeitet werden, entscheidet die aufrufende Umgebung.

## Verhältnis zu SimulationRunner

Ein `SimulationRunner` führt einen vollständigen Simulationslauf über `HopperSystem2D` aus und liefert diesen als `SimulationResult`.

Die Visualisierung verwendet dieses Ergebnis anschließend als Input.

Der Datenfluss lautet damit:

```text
SimulationRunner
        ↓
SimulationResult
        ↓
Visualisierung
        ↓
Plots
```

Die Visualisierung selbst kennt weder den `SimulationRunner` noch das konkrete `HopperSystem2D`.

## Erste Zielsetzung

Die Visualisierungsschicht dient in erster Linie dazu, das Simulationsverhalten schnell inspizieren und plausibilisieren zu können.

Sie ist noch kein allgemeines Reporting- oder Analyseframework.

Mit den Basisplots kann das Starrkörperverhalten untersucht werden.

Mit den Vergleichsplots kann zusätzlich geprüft werden, wie stark sich angeforderte Commands und tatsächlich erreichbare Actuation voneinander unterscheiden.

Eine konkrete Auswertung ist im Beispiel [Open-Loop mit konstantem Gimbal-Command](examples/open_loop_constant_gimbal.md) beschrieben.
