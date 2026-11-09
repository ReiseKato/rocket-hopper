# Numerische Integration

Die aktuelle Implementierung enthält absichtlich noch keinen produktiven Integrator. Diese Trennung verhindert, dass die Physik mit einer bestimmten Integrationsmethode gekoppelt wird.

Die Dynamik liefert

\[
\dot x=f(x,u,p).
\]

Ein Integrator erzeugt daraus diskrete Zustände

\[
x_k\rightarrow x_{k+1}.
\]

## Euler als Referenz

Der explizite Euler-Schritt lautet

\[
x_{k+1}=x_k+f(x_k,u_k,p)\Delta t.
\]

Euler ist vor allem als leicht nachvollziehbare Referenz nützlich.

## Geplante erste produktive Methode

Für die erste tatsächliche Simulationsschleife ist ein klassischer Runge-Kutta-Integrator vierter Ordnung (RK4) vorgesehen. Die Schrittweite \(\Delta t\) bleibt konfigurierbar.

Vor dem Einsatz in Controllerstudien wird geprüft, ob relevante Ergebnisse bei einer Verringerung von \(\Delta t\) ausreichend konvergieren.

## Zustandsdarstellung innerhalb eines Integrators

Die Dynamikschnittstelle arbeitet mit den semantischen Modellklassen des Simulators.

Konzeptionell gilt:

\[
\dot{x}=f(x,u,p).
\]

Dabei wird der Zustand als `HopperState2D` und die resultierende Ableitung als `HopperStateDerivative2D` dargestellt.

Numerische Integrationsverfahren müssen jedoch Operationen wie

\[
x+\Delta t\,\dot{x}
\]

oder im Fall von RK4 beispielsweise

\[
x_{\mathrm{mid}}
=
x_k
+
\frac{\Delta t}{2}k_1
\]

berechnen können.

Für solche Operationen darf der Integrator Zustand und Ableitung intern in NumPy-Vektoren umwandeln.

Die grundsätzliche Datenflussgrenze lautet:

```text
HopperState2D
      │
      │ to_vector()
      ▼
NumPy-Vektor
      │
      │ numerische Integration
      ▼
NumPy-Vektor
      │
      │ from_vector()
      ▼
HopperState2D
```

Ein Integrator nimmt daher einen semantischen Hopperzustand entgegen und liefert ebenfalls wieder einen `HopperState2D` zurück.

Die numerische Vektordarstellung bleibt ein internes Implementierungsdetail des Integrators.

### Konstante Actuation während eines Integrationsschritts

Für die erste Version des Simulators wird angenommen, dass die physikalische Actuation während eines einzelnen Integrationsschritts konstant bleibt. Diese Annahme entspricht einem Zero-Order Hold.

Während beispielsweise ein RK4-Integrator mehrere Zwischenzustände innerhalb eines Zeitschritts auswertet, wird für alle diese Auswertungen dieselbe physikalische Actuation verwendet.

Das bedeutet insbesondere, dass Controller und Aktuatormodelle nicht innerhalb eines einzelnen Integrationsschritts erneut ausgewertet werden.

Diese Annahme kann später erweitert werden, falls Multi-Rate-Simulation, detaillierte Aktuatordynamik oder unterschiedliche Abtastraten relevant werden.


## Integrator-Schnittstelle

Numerische Integratoren sind im Fast Simulator bewusst von der Hopper-Dynamik getrennt.

Ein Integrator arbeitet ausschließlich mit numerischen Zustandsvektoren und einer Ableitungsfunktion der Form

\[
\dot{x}=f(t,x).
\]

Die gemeinsame Schnittstelle wird durch die abstrakte Klasse `Integrator` definiert.

Ein Integrationsschritt besitzt konzeptionell folgende Eingaben:

```text
derivative    Ableitungsfunktion f(t, x)
state         numerischer Zustandsvektor x
time          aktuelle Simulationszeit [s]
dt            Integrationsschrittweite [s]
```

und liefert den numerischen Zustand nach einem Zeitschritt zurück.

```text
Integrator.step(...)
        ↓
numerischer Zustand bei t + dt
```

### Trennung von Integrator und Hopper-Modell

Der Integrator kennt keine Hopper-spezifischen Klassen oder physikalischen Größen.

Insbesondere kennt er nicht:

- `HopperState2D`
- `HopperActuation2D`
- `Hopper2DDynamics`
- Masse oder Trägheitsmoment
- Schub oder Gimbalwinkel

Die Verbindung zwischen dem semantischen Hoppermodell und einem numerischen Integrator wird von einer darüberliegenden Simulationsschicht hergestellt.

Dort wird ein `HopperState2D` zunächst über `to_vector()` in die numerische Zustandsdarstellung überführt.

Die Hopper-Dynamik wird anschließend über eine numerische Ableitungsfunktion verfügbar gemacht.

Nach dem Integrationsschritt wird der numerische Zustandsvektor wieder über `HopperState2D.from_vector()` in die semantische Zustandsdarstellung überführt.

Der Datenfluss ist damit:

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
      ▼
numerischer Zustandsvektor
      │
      │ from_vector()
      ▼
HopperState2D
```

Diese Trennung ermöglicht es, Integratoren unabhängig von der Hopperphysik zu entwickeln, zu testen und auszutauschen.

### Explizite Zeitabhängigkeit

Die Integrator-Schnittstelle verwendet bewusst die allgemeine Form

\[
\dot{x}=f(t,x)
\]

und nicht nut

\[
\dot{x}=f(x).
\]

Das aktuelle 2D-Hoppermodell besitzt noch keine explizite Zeitabhängigkeit.

Die Zeit wird dennoch bereits Teil der Schnittstelle, damit später beispielsweise zeitabhängige Störungen oder Umgebungsmodelle unterstützt werden können, ohne die Integrator-API zu verändern.

Dies ist insbesondere für mehrstufige Verfahren wie RK4 relevant, da dort Ableitungen an unterschiedlichen Zeitpunkten innerhalb eines Integrationsschritts ausgewertet werden.

### Actuation während eines Integrationsschritts

Für die aktuelle Simulatorstufe bleibt die bereits definierte Zero-Order-Hold-Annahme bestehen. Die physikalische Actuation wird während eines vollständigen Integrationsschritts konstant gehalten. Die Hopper-spezifische Ableitungsfunktion kapselt diese konstante Actuation. Der numerische Integrator selbst kennt die Actuation nicht.

### Referenzintegrator

EulerIntegrator dient als einfache Referenzimplementation der gemeinsamen Integrator-Schnittstelle.

Er soll insbesondere:

- die Integrator-API verifizieren,
- einfache Tests ermöglichen,
- als leicht nachvollziehbare Vergleichsimplementation dienen.

Für die mathematischen Grundlagen des expliziten Euler-Verfahrens wird auf externe Literatur verwiesen, beispielsweise auf den Wikipedia-Artikel zum expliziten Euler-Verfahren:

[Explizites Euler Verfahren](https://de.wikipedia.org/wiki/Explizites_Euler-Verfahren)

Eine eigene Herleitung des Verfahrens ist nicht Bestandteil dieser Projektdokumentation.

Für die eigentliche Fast-Simulator-Nutzung ist später ein höherwertiges Integrationsverfahren wie RK4 vorgesehen.

### RK4-Integrator

`RK4Integrator` implementiert das klassische Runge-Kutta-Verfahren vierter Ordnung über dieselbe allgemeine `Integrator`-Schnittstelle wie `EulerIntegrator`.

Eine mathematische Herleitung des Verfahrens ist nicht Bestandteil dieser Projektdokumentation.

Als externe Einführung kann beispielsweise verwendet werden:

https://de.wikipedia.org/wiki/Runge-Kutta-Verfahren

Für das Rocket-Hopper-Projekt sind folgende Implementierungsdetails relevant.

#### Verwendung der Simulationszeit

Die allgemeine Ableitungsfunktion des Integrators besitzt die Form

\[
\dot{x}=f(t,x).
\]

`RK4Integrator` wertet diese Funktion innerhalb eines Integrationsschritts an den für das klassische RK4-Verfahren erforderlichen Zwischenzeiten aus.

Damit unterstützt die gemeinsame Integrator-Schnittstelle auch zukünftige Modelle mit expliziter Zeitabhängigkeit.

Das aktuelle `Hopper2DDynamics` verwendet die Simulationszeit noch nicht explizit.

#### Actuation innerhalb eines RK4-Schritts

Die bereits definierte Zero-Order-Hold-Annahme bleibt auch bei RK4 bestehen.

Für einen Aufruf von

```python
simulator.step(
    actuation=actuation,
    dt=dt,
)
```

wird dieselbe `HopperActuation2D` während aller internen Ableitungsauswertungen des RK4-Verfahrens verwendet. Damit gilt weiterhin die Zero-Order-Hold-Annahme.

Der RK4-Integrator selbst kennt die Actuation nicht. Sie wird weiterhin durch die vom `HopperSimulator` bereitgestellte Ableitungsfunktion gekapselt.

#### Austauschbarkeit

RK4Integrator verwendet exakt dieselbe öffentliche Schnittstelle wie EulerIntegrator.

Daher kann der Integrator beim Erzeugen eines HopperSimulator ausgetauscht werden, ohne das Dynamikmodell oder die Simulatorlogik zu ändern:

```python
simulator = HopperSimulator(
    dynamics=dynamics,
    integrator=RK4Integrator(),
    initial_state=initial_state,
)
```

`EulerIntegrator` bleibt als einfache Referenzimplementation erhalten.

`RK4Integrator` ist für die weitere Entwicklung des Fast Simulators der bevorzugte Standardintegrator, solange kein konkreter Grund für ein anderes Verfahren besteht.