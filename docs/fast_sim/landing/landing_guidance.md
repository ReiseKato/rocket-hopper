# Landing Guidance

## Zweck

Die Landing Guidance erzeugt während eines Landemanövers die Referenzen für die bereits vorhandenen Regelkreise.

Sie berechnet keine Aktuator-Commands.

Ihre Aufgabe lautet:

\[
(\text{Zeit},\text{Zustand},\text{Landing-Konfiguration})
\rightarrow
\begin{cases}
\text{HorizontalPositionReference2D} \\
\text{AltitudeReference2D}
\end{cases}
\]

Damit bleibt die Trennung zwischen Guidance und Control erhalten.

```text
Landing Guidance
      │
      ├── HorizontalPositionReference2D
      │              ↓
      │    Horizontal Position Controller
      │              ↓
      │       Attitude Reference
      │              ↓
      │       Attitude Controller
      │              ↓
      │          delta_cmd
      │
      └── AltitudeReference2D
                     ↓
             Altitude Controller
                     ↓
                 thrust_cmd
```

## Ziel der ersten Version

Die erste Landing Guidance soll einen einfachen, reproduzierbaren Sinkflug erzeugen.

Sie besitzt drei Phasen:

```text
APPROACH
   ↓
FLARE
   ↓
FINAL
```

Die horizontale Zielposition bleibt während des gesamten Manövers konstant.

Die vertikale Referenz bewegt sich dagegen mit der Zeit in Richtung Boden.

## Schwerpunkt-Höhe beim Touchdown

Die Guidance verwendet die minimale Fahrwerksgeometrie.

Für einen aufrechten Hopper liegt der Schwerpunkt beim geometrischen Bodenkontakt bei:

\[
p_{z,\mathrm{touchdown}}
=
z_{\mathrm{ground}}
+
h
\]

mit:

\[
h=\text{cg\_to\_feet}.
\]

Die Guidance fährt die vertikale Referenz deshalb nicht gegen:

\[
p_z=0,
\]

sondern gegen die geometrisch sinnvolle Schwerpunkt-Höhe beim Touchdown.

## Horizontale Referenz

Während des gesamten Landemanövers gilt:

\[
p_{x,\mathrm{ref}}
=
p_{x,\mathrm{target}}
\]

und:

\[
v_{x,\mathrm{ref}}
=
0.
\]

Der bestehende `HorizontalPositionController2D` erzeugt daraus die notwendige Lagereferenz.

Damit kann der Hopper auch während des Sinkflugs horizontal zur Landezone korrigieren.

## Approach-Phase

Oberhalb der Flare-Höhe bewegt sich die vertikale Referenz mit konstanter Sinkrate.

Der Konfigurationsparameter:

\[
v_{\mathrm{approach}}>0
\]

wird als positive Geschwindigkeitsgröße angegeben.

Da im World Frame positive \(z\)-Richtung nach oben zeigt, lautet die tatsächlich erzeugte Referenz:

\[
v_{z,\mathrm{ref}}
=
-v_{\mathrm{approach}}.
\]

Für die Positionsreferenz gilt:

\[
p_{z,\mathrm{ref}}(t)
=
p_{z,0}
-
v_{\mathrm{approach}}t.
\]

Die Approach-Phase endet bei:

\[
p_{z,\mathrm{flare}}
=
p_{z,\mathrm{touchdown}}
+
h_{\mathrm{flare}}.
\]

## Flare-Phase

In der Flare-Phase soll die Sinkgeschwindigkeit von:

\[
-v_{\mathrm{approach}}
\]

auf:

\[
-v_{\mathrm{touchdown}}
\]

reduziert werden.

Gleichzeitig bewegt sich die Positionsreferenz von:

\[
p_{z,\mathrm{flare}}
\]

bis:

\[
p_{z,\mathrm{touchdown}}.
\]

Die erste Implementierung verwendet dafür eine kubische Hermite-Trajektorie.

Damit sind sowohl:

\[
p_{z,\mathrm{ref}}
\]

als auch:

\[
v_{z,\mathrm{ref}}
\]

am Übergang zwischen Approach und Flare stetig.

Die Flare-Dauer wird so gewählt, dass die mittlere Geschwindigkeit der beiden Randgeschwindigkeiten zur zurückzulegenden Flare-Distanz passt:

\[
T_{\mathrm{flare}}
=
\frac{
2h_{\mathrm{flare,actual}}
}{
v_{\mathrm{approach}}
+
v_{\mathrm{touchdown}}
}.
\]

Falls die Landing Guidance bereits innerhalb der nominalen Flare-Zone gestartet wird, wird die tatsächliche Flare-Distanz entsprechend verkürzt.

## Final-Phase

Nach Abschluss des geplanten Flare-Profils bleibt die Referenz bei:

\[
p_{z,\mathrm{ref}}
=
p_{z,\mathrm{touchdown}}
\]

und:

\[
v_{z,\mathrm{ref}}
=
-v_{\mathrm{touchdown}}.
\]

Die Guidance beendet die Simulation nicht.

Die negative Touchdown-Geschwindigkeitsreferenz bleibt bestehen, bis eine übergeordnete Landing-Orchestrierung den tatsächlichen Bodenkontakt über `TouchdownDetector2D` erkennt.

Dadurch wird vermieden, dass die Guidance selbst Kontaktlogik übernimmt.

## Warum die Referenz nicht einfach auf den Boden springt

Eine direkte Vorgabe:

\[
p_{z,\mathrm{ref}}
=
p_{z,\mathrm{touchdown}}
\]

bereits zu Beginn des Sinkflugs würde beim vorhandenen PD-Altitude-Controller einen großen Positionsfehler erzeugen.

Der daraus entstehende Schub-Command könnte zu einer unnötig aggressiven Abwärtsbeschleunigung führen.

Die Landing Guidance erzeugt deshalb eine bewegte vertikale Referenz.

Der Altitude Controller verfolgt damit eine Trajektorie statt eines weit entfernten statischen Endpunkts.

## Softwarekomponenten

### `LandingGuidance2D`

`LandingGuidance2D` definiert die gemeinsame Guidance-Schnittstelle:

```python
compute_references(
    time,
    state,
) -> LandingGuidanceReference2D
```

Der aktuelle `HopperState2D` ist Bestandteil der Schnittstelle, auch wenn die erste konkrete Implementation ihr vertikales Profil zeitbasiert erzeugt.

Spätere Guidance-Algorithmen können damit direkt zustandsabhängig arbeiten.

### `PiecewiseLandingGuidance2D`

`PiecewiseLandingGuidance2D` implementiert das erste einfache Landing-Profil:

```text
konstante Approach-Sinkrate
        ↓
kubische Flare-Trajektorie
        ↓
finale Touchdown-Referenz
```

Die Klasse besitzt keine Controller und keine Aktuatormodelle.

### `LandingGuidanceParameters2D`

Die Konfiguration enthält:

- `target_x`,
- `ground_height`,
- `approach_descent_speed`,
- `flare_height`,
- `touchdown_descent_speed`.

Sinkgeschwindigkeiten werden als positive Beträge konfiguriert.

Die Guidance erzeugt daraus negative \(v_z\)-Referenzen.

### `LandingGuidanceReference2D`

Die Ausgabe enthält:

- die aktuelle `LandingPhase2D`,
- eine `HorizontalPositionReference2D`,
- eine `AltitudeReference2D`.

## Phasen

`LandingPhase2D` besitzt:

```python
LandingPhase2D.APPROACH
LandingPhase2D.FLARE
LandingPhase2D.FINAL
```

Die Phase dient zunächst der Nachvollziehbarkeit und später der Auswertung beziehungsweise Visualisierung.

Sie verändert nicht die Plant.

## Beispielparameter

Für einen ersten Versuch könnten beispielsweise verwendet werden:

\[
p_{x,\mathrm{target}}=0
\]

\[
z_{\mathrm{ground}}=0
\]

\[
v_{\mathrm{approach}}=0.5\,\mathrm{m/s}
\]

\[
h_{\mathrm{flare}}=0.5\,\mathrm m
\]

\[
v_{\mathrm{touchdown}}=0.1\,\mathrm{m/s}.
\]

Diese Werte sind Simulationsparameter und noch keine Hardware- oder Sicherheitsgrenzen.

## Verantwortungsgrenze

Die Landing Guidance kennt keine:

- Schub-Commands,
- Gimbal-Commands,
- Aktuatorgrenzen,
- Kontaktkräfte,
- Landing-Erfolgskriterien,
- Simulationsabbruchlogik.

Sie darf Referenzen erzeugen, aber keine physikalische Actuation direkt bestimmen.

Der vollständige geplante Ablauf lautet:

```text
Landing Guidance
        ↓
Controller References
        ↓
Controller Stack
        ↓
HopperActuationCommand2D
        ↓
HopperSystem2D
        ↓
TouchdownDetector2D
        ↓
Landing-Orchestrierung
```

## Verhältnis zur Touchdown Detection

Guidance und Touchdown Detection bleiben getrennt.

Die Guidance bestimmt:

> Wohin und mit welcher Sollgeschwindigkeit soll sich der Hopper bewegen?

Die Touchdown Detection bestimmt:

> Hat ein Fuß den Boden geometrisch erreicht?

Damit kann später eine übergeordnete Landing-Simulation bei erkanntem Touchdown stoppen und den tatsächlichen Zustand bewerten.

## Grenzen der ersten Version

Die erste Guidance berücksichtigt noch keine:

- optimale Trajektorienplanung,
- Treibstoffminimierung,
- maximale Beschleunigungen,
- Jerk-Begrenzung,
- Hindernisse,
- Wind,
- dynamische Auswahl einer Landezone,
- zustandsabhängige Neuplanung,
- explizite Touchdown-Zeitoptimierung,
- Kontaktphysik.

Sie dient zunächst dazu, den vorhandenen Controller-Stack mit einer physikalisch sinnvollen, zeitlich veränderlichen Landereferenz zu versorgen.
