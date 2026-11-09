# ADR 0006: Zustandsbehaftete Simulator-Schicht

## Status

Akzeptiert

## Kontext

Das Hopper-Dynamikmodell und die numerischen Integratoren wurden
bewusst als zustandslose Komponenten entworfen.

`Hopper2DDynamics` beschreibt ausschließlich die kontinuierliche
Abbildung

\[
(x,u,p)\rightarrow\dot{x}.
\]

Ein Integrator beschreibt die numerische Abbildung eines Zustands auf
den nächsten Zustand.

Für einen laufenden Simulationsprozess muss jedoch zusätzlich verwaltet
werden, welcher Zustand aktuell ist und zu welchem Simulationszeitpunkt
dieser Zustand gehört.

Diese Verantwortung soll weder in das Dynamikmodell noch in einen
numerischen Integrator verschoben werden.

## Entscheidung

Es wird eine separate, zustandsbehaftete Simulator-Schicht eingeführt.

`HopperSimulator` verwaltet:

- den aktuellen `HopperState2D`,
- die aktuelle Simulationszeit,
- eine Instanz von `Hopper2DDynamics`,
- eine Instanz der gemeinsamen `Integrator`-Schnittstelle.

Ein Aufruf von `step()` führt genau einen vollständigen
Integrationsschritt aus und aktualisiert anschließend Zustand und Zeit.

Das Dynamikmodell und die Integratoren bleiben weiterhin zustandslos.

Die physikalische Actuation wird während eines Integrationsschritts
konstant gehalten und innerhalb der vom Integrator verwendeten
Ableitungsfunktion gekapselt.

Zustand und Zeit werden erst aktualisiert, nachdem der vollständige
Integrationsschritt erfolgreich abgeschlossen wurde.

## Konsequenzen

### Positive Konsequenzen

- Laufender Simulationszustand und mathematische Modelle bleiben klar
  voneinander getrennt.
- Dynamik und Integratoren bleiben unabhängig und einfach testbar.
- Der Integrator kann ohne Änderung der Simulatorlogik ausgetauscht
  werden.
- Ein fehlgeschlagener Integrationsschritt hinterlässt keinen teilweise
  aktualisierten Simulationszustand.
- Die Konvertierung zwischen semantischem Hopperzustand und numerischer
  Integratordarstellung befindet sich an einer klaren Stelle.
- Die Simulator-Schicht bildet eine natürliche spätere
  Orchestrierungsgrenze für weitere Komponenten.

### Negative Konsequenzen

- `HopperSimulator` besitzt im Gegensatz zu Dynamics und Integratoren
  veränderlichen internen Zustand.
- Für unabhängige Simulationsläufe werden separate Simulatorinstanzen
  benötigt.
- Die Simulator-Schicht erzeugt zusätzliche Konvertierungen zwischen
  `HopperState2D` und numerischen Zustandsvektoren.

## Abgrenzung

`HopperSimulator` ist keine allgemeine Experiment- oder
Applikationsklasse.

Insbesondere werden Controller, Logging, Visualisierung,
Aktuatormodelle und Experimentkonfiguration nicht Teil dieser Klasse,
solange dafür kein konkreter architektonischer Bedarf besteht.