# ADR 0005: Trennung von numerischer Integration und Hopper-Dynamik

## Status

Akzeptiert

## Kontext

Das kontinuierliche Hoppermodell beschreibt die Zustandsableitung

\[
\dot{x}=f(x,u,p).
\]

Für eine zeitdiskrete Simulation muss diese kontinuierliche Dynamik numerisch integriert werden.

Mehrere Integrationsverfahren sollen unterstützt und unabhängig voneinander entwickelt, getestet und ausgetauscht werden können.

Eine direkte Kopplung des Integrators an Klassen wie `Hopper2DDynamics`, `HopperState2D` oder `HopperActuation2D` würde numerische Verfahren unnötig an das aktuelle 2D-Hoppermodell binden.

Langfristig soll dieselbe Integrationsinfrastruktur auch für andere Zustandsmodelle, insbesondere einen späteren 3D-Hopper, verwendet werden können.

## Entscheidung

Numerische Integratoren werden als eigenständige, Hopper-unabhängige Komponenten implementiert.

Die gemeinsame Schnittstelle wird durch die abstrakte Klasse `Integrator` definiert.

Ein Integrator arbeitet mit:

- einer numerischen Ableitungsfunktion \(f(t,x)\),
- einem numerischen Zustandsvektor,
- der aktuellen Simulationszeit,
- einer Integrationsschrittweite \(\Delta t\).

Der Integrator nimmt keine Hopper-spezifischen Modellobjekte entgegen.

Die Übersetzung zwischen `HopperState2D` und der numerischen Zustandsdarstellung wird von einer darüberliegenden Simulationsschicht übernommen.

Die Hopper-Dynamik wird dort in eine numerische Ableitungsfunktion eingebettet.

Die allgemeine Ableitungsfunktion besitzt bewusst die Form

\[
\dot{x}=f(t,x),
\]

auch wenn das aktuelle Hoppermodell noch keine explizite Zeitabhängigkeit besitzt.

Die physikalische Actuation wird während eines einzelnen Integrationsschritts konstant gehalten und ist kein Bestandteil der Integrator-Schnittstelle.

## Konsequenzen

### Positive Konsequenzen

- Integratoren sind vollständig unabhängig von der Hopperphysik testbar.
- Euler, RK4 und spätere Verfahren können über dieselbe Schnittstelle ausgetauscht werden.
- Die Integrationsinfrastruktur kann später auch für andere Zustandsmodelle verwendet werden.
- `Hopper2DDynamics` bleibt frei von numerischer Integrationslogik.
- Der Integrator kennt keine Controller-, Aktuator- oder Fahrzeugparameter.
- Zeitabhängige Modelle können später unterstützt werden, ohne die Integrator-Schnittstelle zu ändern.

### Negative Konsequenzen

- Zwischen dem semantischen Hopperzustand und dem Integrator ist eine Adapter- beziehungsweise Simulationsschicht notwendig.
- Zustände müssen an der Integrationsgrenze zwischen `HopperState2D` und NumPy-Vektoren konvertiert werden.
- Die Hopper-spezifische Actuation muss beim Aufbau der numerischen Ableitungsfunktion gekapselt werden.

## Referenzimplementation

`EulerIntegrator` dient als einfache Referenzimplementation der Schnittstelle.

Er ist nicht als endgültige Wahl des Integrationsverfahrens für den Fast Simulator festgelegt.

Ein höherwertiger Integrator wie RK4 soll später über dieselbe Schnittstelle implementiert werden.