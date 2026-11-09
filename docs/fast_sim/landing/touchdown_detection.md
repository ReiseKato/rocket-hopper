# Touchdown Detection

## Zweck

Die Touchdown Detection bestimmt, ob mindestens einer der beiden Fußpunkte der minimalen Landegeometrie den Boden erreicht hat.

Sie baut auf `LandingGearGeometry2D` auf und ergänzt eine flache Bodenebene.

Die Schicht modelliert noch keine Kontaktkräfte.

Ihre Aufgabe lautet ausschließlich:

\[
(\text{Geometrie},\text{Boden},HopperState2D)
\rightarrow
\text{Kontaktzustand}.
\]

## Bodenmodell

Der Boden wird zunächst als horizontale Ebene beschrieben:

\[
z=z_{\mathrm{ground}}.
\]

`ground_height` ist damit eine konstante Weltkoordinate.

Für die ersten Landeversuche kann verwendet werden:

\[
z_{\mathrm{ground}}=0.
\]

Die Implementierung erlaubt jedoch auch andere endliche Bodenhöhen.

## Bodenfreiheit

Für beide Fußpunkte wird die vertikale Bodenfreiheit berechnet:

\[
c_L
=
z_L-z_{\mathrm{ground}}
\]

und:

\[
c_R
=
z_R-z_{\mathrm{ground}}.
\]

Dabei gilt:

\[
c>0
\]

für einen Fuß oberhalb des Bodens,

\[
c=0
\]

für geometrischen Erstkontakt und:

\[
c<0
\]

wenn der numerische Zustand den Fuß bereits unterhalb der Bodenebene enthält.

Da noch keine Kontaktphysik existiert, wird eine negative Bodenfreiheit nicht korrigiert.

Sie wird ausschließlich als erkannter Kontakt ausgewertet.

## Kontaktbedingung

Ein Fuß gilt als in Kontakt, wenn:

\[
c\leq0.
\]

Damit existieren vier mögliche Ergebnisse:

```text
NONE
LEFT
RIGHT
BOTH
```

`NONE` bedeutet, dass beide Füße oberhalb des Bodens liegen.

`LEFT` bedeutet, dass nur der linke Fuß den Boden erreicht hat.

`RIGHT` bedeutet, dass nur der rechte Fuß den Boden erreicht hat.

`BOTH` bedeutet, dass beide Fußpunkte auf oder unterhalb der Bodenebene liegen.

## Softwarekomponenten

### `TouchdownDetector2D`

Der Detector kombiniert:

- `LandingGearGeometry2D`,
- `ground_height`.

Er ist zustandslos.

Ein Aufruf:

```python
detection = detector.detect(state)
```

wertet ausschließlich den übergebenen `HopperState2D` aus.

Der Detector verändert weder den Hopperzustand noch die Simulation.

### `TouchdownDetection2D`

Das Ergebnis enthält:

- den Kontaktzustand,
- die Weltpositionen beider Füße,
- die Bodenhöhe,
- die Bodenfreiheit des linken Fußes,
- die Bodenfreiheit des rechten Fußes.

Zusätzlich stehen abgeleitete Eigenschaften zur Verfügung:

```python
detection.is_touchdown
detection.left_in_contact
detection.right_in_contact
detection.minimum_clearance
```

### `TouchdownContact2D`

Der Kontaktzustand wird als Enum beschrieben:

```python
TouchdownContact2D.NONE
TouchdownContact2D.LEFT
TouchdownContact2D.RIGHT
TouchdownContact2D.BOTH
```

Damit muss aufrufender Code keine eigenen Kombinationen aus zwei booleschen Werten interpretieren.

## Verhalten bei aufrechter Lage

Für einen aufrechten Hopper gilt:

\[
\theta=0.
\]

Beide Füße liegen dann bei:

\[
z_L=z_R=p_z-h.
\]

Bei:

\[
z_{\mathrm{ground}}=0
\]

erfolgt geometrischer Touchdown deshalb bei:

\[
p_z=h.
\]

Beide Füße erreichen den Boden gleichzeitig:

```text
BOTH
```

## Verhalten bei positiver Lage

Bei:

\[
\theta>0
\]

liegt der rechte Fuß tiefer.

Bei einem langsam sinkenden Hopper wird deshalb zuerst:

```text
RIGHT
```

erkannt.

Für:

\[
\theta<0
\]

gilt entsprechend:

```text
LEFT
```

für den ersten geometrischen Fußkontakt.

## Keine Kontaktphysik

Die Touchdown Detection erzeugt keine Kontaktkraft.

Sie verändert insbesondere nicht:

- \(p_z\),
- \(v_z\),
- \(\theta\),
- \(\omega\).

Ein numerischer Zustand mit:

\[
c<0
\]

wird daher nicht automatisch auf die Bodenebene projiziert.

Das ist bewusst.

Kontaktphysik ist eine eigene spätere Modellierungsaufgabe.

## Noch kein automatischer Simulationsabbruch

`TouchdownDetector2D` beendet keinen `SimulationRunner`.

Damit bleibt die Verantwortungsgrenze klar:

```text
HopperSystem2D
        ↓
neuer HopperState2D
        ↓
TouchdownDetector2D
        ↓
TouchdownDetection2D
        ↓
übergeordnete Landing-Orchestrierung
```

Die spätere Landing-Orchestrierung kann entscheiden, ob bei:

```python
detection.is_touchdown
```

der Versuch beendet und der Touchdown-Zustand bewertet werden soll.

## Diskrete Simulation

Die Simulation arbeitet mit endlichen Zeitschritten.

Daher ist es möglich, dass ein Fuß zwischen zwei Zuständen die Bodenebene überquert.

Der erste gespeicherte Zustand mit erkanntem Kontakt kann dann bereits eine kleine negative Bodenfreiheit besitzen.

Die erste Version akzeptiert diese Diskretisierung.

Eine spätere Event-Lokalisierung könnte den exakten Zeitpunkt des Nulldurchgangs innerhalb eines Integrationsschritts genauer bestimmen.

Für den ersten 2D-Landeversuch ist dies bewusst noch nicht Bestandteil der Touchdown Detection.

## Verantwortungsgrenze

Die Touchdown Detection kennt keine:

- Landing Guidance,
- Controller,
- Erfolgskriterien,
- maximal zulässigen Aufprallgeschwindigkeiten,
- Kontaktkräfte,
- Reibung,
- Federung,
- Abbruchstrategie.

Sie beantwortet nur:

> Hat einer der geometrischen Fußpunkte die definierte Bodenebene erreicht?

Die Bewertung, ob dieser Touchdown eine erfolgreiche Landung darstellt, erfolgt in einer separaten Landing-Evaluation.
