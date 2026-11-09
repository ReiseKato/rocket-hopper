# Simulationsläufe

## Zweck

`SimulationRunner` führt ein `HopperSystem2D` über mehrere aufeinanderfolgende Systemschritte aus.

Der Runner enthält keine eigene Fahrzeugphysik, keine Aktuatordynamik und kein numerisches Integrationsverfahren.

Er koordiniert ausschließlich:

- die wiederholte Erzeugung von Commands,
- die wiederholte Ausführung von `HopperSystem2D.step(...)`,
- die Aufzeichnung der entstehenden Verlaufsdaten in einem `SimulationResult`.

Die Verantwortlichkeiten sind damit:

```text
CommandProvider
    erzeugt den Command für den nächsten Schritt

HopperSystem2D
    führt einen vollständigen physischen Systemschritt aus

SimulationRunner
    koordiniert mehrere Systemschritte

SimulationResult
    speichert das unveränderliche Ergebnis eines Laufs
```

## Ablauf eines Systemschritts

Vor jedem Systemschritt wird eine `CommandProvider`-Funktion aufgerufen.

Sie erhält:

- die aktuelle Simulationszeit,
- den aktuellen Starrkörperzustand `HopperState2D`.

Die Schnittstelle entspricht konzeptionell:

\[
(t_k,x_{\mathrm{RB},k})
\rightarrow
u_{\mathrm{cmd},k}.
\]

Der zurückgegebene `HopperActuationCommand2D` wird anschließend an `HopperSystem2D` übergeben.

Dort werden zunächst die Aktuatormodelle fortgeschrieben.

Aus dem Command entsteht die tatsächlich wirkende `HopperActuation2D`.

Anschließend führt der enthaltene `HopperSimulator` den Starrkörper-Integrationsschritt aus.

Der Ablauf lautet:

```text
aktuelles t_k und x_RB,k
          │
          ▼
    CommandProvider
          │
          ▼
HopperActuationCommand2D
          │
          ▼
    HopperSystem2D
          │
          ├── Aktuatormodelle
          │       ↓
          │  HopperActuation2D
          │
          └── HopperSimulator
                  ↓
             x_RB,k+1
```

Der `CommandProvider` wird erst vor dem nächsten Systemschritt erneut ausgewertet.

## Bedeutung des Command Providers

`CommandProvider` beschreibt, woher der Command für den jeweils nächsten Systemschritt stammt.

Er ist keine physische Komponente des Hoppers.

Für einen einfachen Open-Loop-Versuch kann beispielsweise immer derselbe `HopperActuationCommand2D` zurückgegeben werden.

Für einen Closed-Loop-Versuch kann ein Controller innerhalb des Providers ausgewertet werden.

Beispielsweise:

```text
aktueller Starrkörperzustand
        +
Attitude Reference
        │
        ▼
AttitudeController2D
        │
        ▼
delta_cmd
        │
        ├── thrust_cmd
        ▼
HopperActuationCommand2D
```

Der Runner selbst kennt weder die konkrete Controllerimplementation noch einzelne Aktuatormodelle.

Er arbeitet ausschließlich mit dem vom Provider erzeugten Command und der öffentlichen Schnittstelle von `HopperSystem2D`.

## Command und physikalische Actuation

Ein Command ist nicht identisch mit der tatsächlich wirkenden Actuation.

Konzeptionell gilt:

\[
u_{\mathrm{cmd},k}
\rightarrow
u_{\mathrm{physical},k}.
\]

Die Umwandlung erfolgt innerhalb von `HopperSystem2D` durch die Aktuatormodelle.

Beispielsweise kann gelten:

\[
\delta_{\mathrm{cmd}}=10^\circ
\]

während der physikalische Gimbal aufgrund eines Rate Limits im selben Systemschritt nur:

\[
\delta=1^\circ
\]

erreicht.

Der `SimulationRunner` zeichnet deshalb beide Größen getrennt auf.

## Zero-Order Hold

Die Aktuatormodelle werden einmal zu Beginn jedes Systemschritts ausgewertet.

Die daraus resultierende physikalische `HopperActuation2D` wird anschließend während des vollständigen Starrkörper-Integrationsschritts konstant gehalten.

Damit gilt innerhalb eines Systemschritts:

\[
u_{\mathrm{physical}}(t)
=
u_{\mathrm{physical},k}
\qquad
\text{für}
\qquad
t_k\leq t<t_{k+1}.
\]

Diese Zero-Order-Hold-Annahme gilt auch dann, wenn der verwendete Integrator intern mehrere Ableitungsauswertungen durchführt, wie beispielsweise RK4.

Der Command wird innerhalb desselben Systemschritts nicht erneut ausgewertet.

## Anzahl der Schritte

Ein Simulationslauf wird aktuell über eine feste Anzahl von Systemschritten beschrieben.

Für:

\[
N
\]

Schritte mit der Schrittweite:

\[
\Delta t
\]

beträgt die simulierte Dauer:

\[
T=N\Delta t.
\]

Diese Schnittstelle wurde bewusst einer direkten Angabe der Dauer vorgezogen.

Dadurch besitzt jeder Systemschritt innerhalb eines Laufs exakt dieselbe Schrittweite.

Eine spätere Experimentkonfiguration kann eine gewünschte Dauer in eine entsprechende Anzahl von Schritten übersetzen.

## SimulationResult

`SimulationResult` speichert den Verlauf eines erfolgreich abgeschlossenen Simulationslaufs.

Für einen Lauf mit \(N\) Systemschritten enthält das Ergebnis:

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

Die Zuordnung lautet:

```text
t0              t1              t2                  tN
●---------------●---------------●------ ... --------●
x0              x1              x2                  xN

    command0        command1                 commandN-1
       ↓               ↓                        ↓
    actuation0      actuation1              actuationN-1
```

`rigid_body_states[k]` beschreibt den Starrkörperzustand zum Zeitpunkt `times[k]`.

`commands[k]` beschreibt den zu Beginn des Systemschritts \(k\) angeforderten `HopperActuationCommand2D`.

`physical_actuations[k]` beschreibt die vom physischen System erzeugte und während des Intervalls

\[
[t_k,t_{k+1})
\]

auf den Starrkörper angewandte `HopperActuation2D`.

Diese Zuordnung muss bei Visualisierung und Analyse erhalten bleiben.

## Systemzustand und Ergebnisdaten

`SimulationResult` zeichnet derzeit auf:

- Starrkörperzustände,
- Commands,
- physikalische Actuations,
- Simulationszeitpunkte.

Interne Zustände einzelner Aktuatormodelle werden nicht als vollständiger allgemeiner Systemzustand gespeichert.

Beispielsweise ist der interne Gimbalzustand zwar über die aufgezeichnete physikalische Actuation beobachtbar, `SimulationResult` ist aber derzeit kein vollständiger serialisierbarer Checkpoint des gesamten simulierten Systems.

Diese Abgrenzung ist bewusst.

## Unveränderlichkeit des Ergebnisses

`SimulationResult` ist unveränderlich.

Ein abgeschlossener Simulationslauf wird nach seiner Erzeugung nicht nachträglich verändert.

Dies erleichtert die Reproduzierbarkeit und verhindert, dass Auswertungs- oder Visualisierungscode versehentlich den gespeicherten Simulationsverlauf verändert.

## Fortsetzen eines Systems

`SimulationRunner.run()` setzt das verwendete `HopperSystem2D` nicht zurück.

Der Runner beginnt immer mit:

- der aktuellen Simulationszeit,
- dem aktuellen Starrkörperzustand,
- den aktuellen internen Zuständen der Aktuatormodelle.

Wird derselbe Runner mehrfach verwendet, beginnt ein neuer Lauf daher genau dort, wo der vorherige Lauf beendet wurde.

Dies betrifft nicht nur den Starrkörper.

Auch ein zustandsbehafteter Gimbal behält beispielsweise seinen zuletzt erreichten Winkel.

Ein vollständig unabhängiger Simulationslauf benötigt daher ein entsprechend neu initialisiertes `HopperSystem2D`.

## Fehler während eines Laufs

Ein vollständiger `run()`-Aufruf ist nicht als atomare Transaktion definiert.

Falls ein Fehler nach bereits erfolgreich ausgeführten Systemschritten auftritt, bleiben diese bereits ausgeführten Schritte im `HopperSystem2D` erhalten.

Das System befindet sich anschließend beim letzten erfolgreich berechneten Zustand.

Der fehlgeschlagene `run()`-Aufruf liefert kein `SimulationResult`.

Falls später ein Bedarf an transaktionalen oder rücksetzbaren Simulationsläufen entsteht, muss diese Funktion explizit ergänzt werden.

## Verhältnis zur Regelung

`SimulationRunner` implementiert selbst keine Controllerlogik.

Ein Controller kann jedoch innerhalb eines `CommandProvider` verwendet werden.

Damit bleibt die Verantwortungsgrenze:

```text
Controller
    erzeugt einen Teil oder die Gesamtheit eines Commands

CommandProvider
    erzeugt HopperActuationCommand2D

SimulationRunner
    koordiniert den Lauf

HopperSystem2D
    simuliert die Plant
```

Die Regelung bleibt damit austauschbar und unabhängig von der physischen Systemschicht.

Weitere Details sind unter [Regelung](control/index.md) dokumentiert.

## Verhältnis zur Visualisierung

`SimulationResult` ist die standardisierte Ausgabe eines Simulationslaufs.

Die Visualisierungsschicht verwendet dieses Ergebnis direkt als Input.

Dadurch bleiben Ausführung und Auswertung getrennt:

```text
SimulationRunner
        ↓
SimulationResult
        ↓
Visualisierung
```

Die Visualisierung verändert das gespeicherte Ergebnis nicht.

Da sowohl Commands als auch physikalische Actuations gespeichert werden, können zusätzlich Vergleiche wie

\[
\delta_{\mathrm{cmd}}(t)
\]

gegen:

\[
\delta(t)
\]

dargestellt werden.

## Abgrenzung

`SimulationRunner` übernimmt bewusst keine:

- Fahrzeugdynamik,
- konkrete Aktuatordynamik,
- Aktuatorbegrenzungen,
- Controllerlogik,
- Experimentkonfiguration,
- Visualisierung,
- Dateispeicherung,
- Auswertung,
- Landing- oder Abbruchlogik.

Diese Funktionen bleiben getrennte Verantwortlichkeiten.
