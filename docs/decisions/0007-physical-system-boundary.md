# ADR 0007: Aktuatormodelle als Bestandteil der physischen Systemschicht

## Status

Akzeptiert

## Kontext

Der Fast Simulator besitzt bisher eine klare Trennung zwischen Starrkörperdynamik, numerischer Integration, Simulation und Ablaufsteuerung.

Mit der Einführung realistischerer Aktuatoren entstehen zusätzliche physikalische Zustände und Begrenzungen.

Insbesondere besitzt ein rate-limitierter Gimbal einen eigenen Zustand.

Diese Aktuatordynamik könnte entweder in den `SimulationRunner`, den `HopperSimulator` oder eine eigene Systemschicht integriert werden.

Eine Integration in den Runner würde physikalische Komponenten mit der Ablaufsteuerung eines Experiments koppeln.

Eine Integration direkt in `HopperSimulator` würde dessen bisher klare Verantwortung für die Starrkörperintegration erweitern und die bestehende Trennung zwischen physikalischer Actuation und deren Erzeugung aufheben.

## Entscheidung

Aktuatormodelle werden als Bestandteil einer eigenen physischen Systemschicht modelliert.

`HopperSystem2D` kombiniert:

- `HopperActuatorSystem2D`,
- `HopperSimulator`.

Der Datenfluss ist:

```text
HopperActuationCommand2D
        ↓
HopperActuatorSystem2D
        ↓
HopperActuation2D
        ↓
HopperSimulator
```

Der `SimulationRunner` soll keine Kenntnisse über einzelne Aktuatormodelle besitzen.

`Hopper2DDynamics` erhält weiterhin ausschließlich die tatsächlich wirkende `HopperActuation2D`.

`HopperState2D` bleibt auf den Starrkörperzustand beschränkt.

Aktuatorzustände verbleiben bei den jeweiligen Aktuatormodellen.

## Konsequenzen

### Positive Konsequenzen

- Physikalische Komponenten bleiben von Experiment- und Runnerlogik getrennt.
- Aktuatormodelle können unabhängig entwickelt und getestet werden.
- Das Starrkörpermodell bleibt unverändert.
- Realistischere Aktuatormodelle können später ausgetauscht werden.
- Zustände bleiben den physikalisch zuständigen Komponenten zugeordnet.
- Die Systemschicht bildet eine klare spätere Plant-Grenze fürController.

### Negative Konsequenzen

- Eine zusätzliche Orchestrierungsschicht wird eingeführt.
- Der vollständige Systemzustand verteilt sich auf mehrere Komponenten.
- Der SimulationRunner und SimulationResult müssen anschließend an die neue Command-/- Systemgrenze angepasst werden.