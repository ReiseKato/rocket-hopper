# Vollständige 2D-Landing-Szenarien

## Zweck

Die drei End-to-End-Szenarien verbinden erstmals die vollständige bisherige Simulationskette:

```text
Rigid-Body Dynamics
        +
Actuator Models
        +
Controller Stack
        +
Landing Guidance
        +
Touchdown Detection
        +
Impact Models
        +
Single-Contact Dynamics
        +
Landing Evaluation
        ↓
vollständiger Landeversuch
```

Es werden keine Fake-Komponenten für die eigentlichen Szenarien verwendet.

Alle drei Beispiele verwenden dieselben physikalischen Parameter, Aktuatormodelle und Controller.

Damit unterscheiden sich die Szenarien primär durch:

- den Anfangszustand,
- die Vorphase vor Aktivierung der Landing Guidance,
- die horizontale Landeposition,
- die Sinkflugparameter.

## Gemeinsame Parameter

Die Beispiele verwenden weiterhin die bisherigen Beispielparameter:

\[
m=2\,\mathrm{kg}
\]

\[
I_y=0.08\,\mathrm{kg\,m^2}
\]

\[
l=0.25\,\mathrm m
\]

sowie:

\[
T_{\max}=30\,\mathrm N.
\]

Der Gimbal besitzt:

\[
|\delta|_{\max}=12^\circ
\]

und:

\[
|\dot\delta|_{\max}=120^\circ/\mathrm s.
\]

Die Fahrwerksgeometrie lautet:

\[
b=0.6\,\mathrm m
\]

und:

\[
h=0.8\,\mathrm m.
\]

Diese Werte bleiben Simulationsparameter und stellen noch keine reale Hardwarefestlegung dar.

## Gemeinsamer Controller Stack

Alle Szenarien verwenden:

```text
HorizontalPositionController2D
        ↓
AttitudeReference2D
        ↓
PDAttitudeController2D
        ↓
gimbal command
```

parallel zu:

```text
PDAltitudeController2D
        ↓
thrust command
```

Der physikalische Gimbal bleibt winkel- und ratenbegrenzt.

Die Landing-Orchestrierung verwendet nach dem ersten Kontakt denselben Aktuatorzustand weiter.

## Szenario 1: Stabilisieren über dem Landeplatz

Datei:

```text
fast_sim/examples/landing_scenario_1_stabilized_vertical.py
```

Der Hopper beginnt bereits mehrere Meter über dem Boden, befindet sich horizontal aber noch nicht über dem vorgesehenen Landeplatz.

Zunächst wird er auf:

\[
p_x=1.5\,\mathrm m
\]

und:

\[
p_z=3.5\,\mathrm m
\]

geregelt.

Die Landing Guidance wird erst aktiviert, nachdem der Hopper diese Position für eine kurze Zeit mit kleinen Geschwindigkeiten, kleinem Lagewinkel und kleiner Winkelgeschwindigkeit gehalten hat.

Die horizontale Landing-Referenz ist identisch mit der vorherigen Staging-Position:

\[
p_{x,\mathrm{landing}}
=
p_{x,\mathrm{staging}}.
\]

Die geplante Landung ist deshalb im Wesentlichen vertikal.

Dieses Szenario dient als einfachster vollständiger Referenzfall.

## Szenario 2: Steigflug und direkte versetzte Landung

Datei:

```text
fast_sim/examples/landing_scenario_2_climb_then_offset_landing.py
```

Der Hopper startet vergleichsweise niedrig bei:

\[
p_z=1.15\,\mathrm m.
\]

Zunächst wird eine Transferposition angeflogen:

\[
p_x=0.8\,\mathrm m
\]

und:

\[
p_z=3.0\,\mathrm m.
\]

Anders als in Szenario 1 wird dort nicht stabilisiert.

Die Vorphase endet bereits beim ersten Erreichen einer Positionstoleranz.

Es gibt keine zusätzliche Forderung nach:

\[
v_x\approx0,
\qquad
v_z\approx0,
\qquad
\omega\approx0.
\]

Die Landing Guidance übernimmt also einen noch bewegten Hopper.

Der eigentliche Landeplatz liegt bei:

\[
p_{x,\mathrm{landing}}
=
2.5\,\mathrm m.
\]

Damit liegt er deutlich rechts von der zuvor angeflogenen Transferposition.

Während des Sinkflugs muss der Hopper deshalb gleichzeitig horizontal weiterfliegen.

Die Landung ist bewusst nicht vertikal.

Dieses Szenario prüft insbesondere, ob die Kaskade aus horizontaler Positionsregelung, Lageregelung und Landing Guidance auch während einer kontinuierlichen Mission funktioniert.

## Szenario 3: Richtungsumkehr aus Querflug

Datei:

```text
fast_sim/examples/landing_scenario_3_crossflight_reversal.py
```

Hier existiert keine separate Vorphase.

Die Landing Guidance ist ab:

\[
t=0
\]

aktiv.

Der Hopper startet in:

\[
p_z=3.2\,\mathrm m
\]

mit einer horizontalen Geschwindigkeit:

\[
v_x=-1.0\,\mathrm{m/s}.
\]

Er bewegt sich also zunächst nach links.

Gleichzeitig zeigt die Nase deutlich nach links:

\[
\theta=-12^\circ.
\]

Der Landeplatz befindet sich dagegen rechts vom Hopper:

\[
p_{x,\mathrm{landing}}
=
1.5\,\mathrm m.
\]

Die Regelung muss daher:

1. die anfängliche Linksbewegung abbremsen,
2. die Fahrzeuglage von deutlich links geneigt auf eine Rechtsbeschleunigung umstellen,
3. die horizontale Bewegungsrichtung umkehren,
4. währenddessen den Sinkflug durchführen,
5. vor Touchdown wieder nahezu aufrechte Lage erreichen.

Dieses Szenario ist der dynamisch anspruchsvollste der drei Referenzfälle.

## Keine Mission-State-Machine in diesen Beispielen

Die Vorphasen der Beispiele werden bewusst noch nicht als allgemeine Mission States implementiert.

Bezeichnungen wie:

```text
Stabilize above landing pad
```

oder:

```text
Climb / transfer — no settling
```

sind zunächst lediglich Szenariobeschreibungen und Visualisierungslabels.

Später kann eine eigene Mission-State-Machine beispielsweise Zustände wie:

```text
START
TARGET_POSITION_REACHED
DESCENT
LANDING_APPROACH
LANDING
```

abbilden.

Diese Ebene soll unabhängig von den physikalischen Landing-Dynamikmodi bleiben:

```text
FREE_FLIGHT
SINGLE_CONTACT
COMPLETED
```

## Visualisierung

Alle drei Beispiele verwenden:

```python
plot_landing_scenario_overview(...)
```

aus:

```text
fast_sim.visualization.landing_scenarios
```

Die Übersicht enthält acht Diagramme.

### Trajektorie

Die \(x\)-\(z\)-Trajektorie zeigt:

- optionale Vorphase,
- Landing-Flug,
- Vorphasenziel,
- horizontales Landing-Ziel,
- Boden,
- ersten Einzelkontakt,
- finalen Zweifußkontakt.

Gerade Szenario 2 und Szenario 3 machen dadurch die horizontale Bewegung während des Sinkflugs sichtbar.

### Flug- und Landing-Phasen

Die Phasenansicht unterscheidet:

```text
optionale Vorphase
Landing / Approach
Landing / Flare
Landing / Final
Single Contact
Completed
```

First Contact und Landing Completion werden zeitlich markiert.

Diese Phasenanzeige ist noch keine allgemeine Mission-State-Machine.

### Fahrzeugzustände

Dargestellt werden:

- horizontale Position,
- vertikale Position,
- horizontale Geschwindigkeit,
- Fahrzeugwinkel.

Damit ist insbesondere bei Szenario 3 die Richtungs- und Lageumkehr sichtbar.

### Impact-Metrik 1: Fußgeschwindigkeiten

Für eine sequentielle Landung werden erster und zweiter Kontakt getrennt dargestellt:

\[
v_n
\]

und:

\[
v_t.
\]

Bei simultanem Kontakt werden die Werte für linken und rechten Fuß angezeigt.

### Impact-Metrik 2: dissipierte Energie

Bei sequentiellem Kontakt werden dargestellt:

\[
E_{\mathrm{diss},1}
\]

und:

\[
E_{\mathrm{diss},2}.
\]

Bei simultanem Kontakt wird die gesamte dissipierte Fahrzeugenergie dargestellt.

Die Energie ist weiterhin eine Größe des idealisierten Kontaktmodells und keine direkt validierte strukturelle Beinbelastung.

## Ausführen

Die Beispiele können einzeln ausgeführt werden:

```bash
python fast_sim/examples/landing_scenario_1_stabilized_vertical.py
```

```bash
python fast_sim/examples/landing_scenario_2_climb_then_offset_landing.py
```

```bash
python fast_sim/examples/landing_scenario_3_crossflight_reversal.py
```

Jedes Beispiel:

1. führt seinen vollständigen Landeversuch aus,
2. schreibt die Kontakt- und Impact-Metriken in die Konsole,
3. öffnet die Landing-Übersicht.

## Bedeutung der Szenarien

Die drei Beispiele bilden eine kleine Progression:

```text
stabiler vertikaler Landing-Fall
        ↓
Landing während horizontalem Transfer
        ↓
Landing nach dynamischer Richtungsumkehr
```

Damit kann die Landing-Implementierung nicht nur an einem nahezu idealen Ausgangszustand geprüft werden.

Sie wird auch mit zunehmend dynamischen Anfangsbedingungen konfrontiert.
