# Minimale Landegeometrie

## Zweck

Für die ersten 2D-Landeversuche reicht es nicht mehr aus, den Hopper ausschließlich als Schwerpunktzustand zu betrachten.

Ein Bodenkontakt bei:

\[
p_z=0
\]

würde bedeuten, dass der Schwerpunkt selbst den Boden erreicht.

Das beschreibt kein physisch sinnvolles Fahrzeug.

Daher erhält der Fast Simulator eine minimale parametrisierte Fahrwerksgeometrie.

Diese Geometrie dient zunächst ausschließlich dazu, die Position zweier möglicher Aufstandspunkte relativ zum Schwerpunkt zu bestimmen.

Sie enthält noch keine Kontaktphysik.

## Modell

Das Fahrwerk wird im 2D-Modell durch zwei symmetrische Fußpunkte beschrieben:

```text
              +z_B
               ↑
               │
               ● CG
              /│\
             / │ \
            /  │ h
           /   │
          ●────┴────●
       linker Fuß  rechter Fuß
          ←── b ──→
```

Verwendet werden zwei Parameter:

\[
b
=
\text{foot\_spacing}
\]

und:

\[
h
=
\text{cg\_to\_feet}.
\]

`foot_spacing` beschreibt den horizontalen Abstand der beiden Fußpunkte voneinander.

`cg_to_feet` beschreibt ihren vertikalen Abstand unterhalb des Schwerpunkts.

Beide Größen sind positive Längen in Metern.

## Fußpunkte im Body Frame

Die Fußpunkte liegen im Body Frame bei:

\[
r_{\mathrm{left}}^B
=
\begin{bmatrix}
-b/2 \\
0 \\
-h
\end{bmatrix}
\]

und:

\[
r_{\mathrm{right}}^B
=
\begin{bmatrix}
+b/2 \\
0 \\
-h
\end{bmatrix}.
\]

Die Geometrie ist damit zunächst:

- symmetrisch,
- starr,
- zweidimensional,
- relativ zum Schwerpunkt definiert.

## Transformation in das World Frame

Für einen Body-Punkt:

\[
(x_B,z_B)
\]

gilt entsprechend der bestehenden Koordinatenkonvention:

\[
x_W
=
p_x
+
x_B\cos\theta
+
z_B\sin\theta
\]

und:

\[
z_W
=
p_z
-
x_B\sin\theta
+
z_B\cos\theta.
\]

Für die beiden Fußpunkte mit:

\[
z_B=-h
\]

folgt:

\[
x_{\mathrm{left}}
=
p_x
-
\frac b2\cos\theta
-
h\sin\theta
\]

\[
z_{\mathrm{left}}
=
p_z
+
\frac b2\sin\theta
-
h\cos\theta
\]

sowie:

\[
x_{\mathrm{right}}
=
p_x
+
\frac b2\cos\theta
-
h\sin\theta
\]

\[
z_{\mathrm{right}}
=
p_z
-
\frac b2\sin\theta
-
h\cos\theta.
\]

## Verhalten bei aufrechter Lage

Für:

\[
\theta=0
\]

liegen beide Füße gleich hoch:

\[
z_{\mathrm{left}}
=
z_{\mathrm{right}}
=
p_z-h.
\]

Ein späterer Bodenkontakt bei einer Bodenhöhe von:

\[
z_{\mathrm{ground}}=0
\]

würde für einen aufrechten Hopper deshalb bei:

\[
p_z=h
\]

auftreten.

Der Schwerpunkt bleibt dabei korrekt oberhalb des Bodens.

## Verhalten bei geneigter Lage

Für:

\[
\theta\neq0
\]

liegen die beiden Füße nicht mehr auf derselben Höhe.

Bei positiver Fahrzeuglage:

\[
\theta>0
\]

liegt im aktuellen Koordinatensystem der rechte Fuß tiefer als der linke.

Damit kann eine spätere Touchdown-Erkennung bestimmen, welcher Fuß zuerst den Boden erreicht.

## Softwarekomponenten

### `LandingGearGeometry2D`

`LandingGearGeometry2D` beschreibt ausschließlich die feste Geometrie.

Die Klasse enthält:

- `foot_spacing`,
- `cg_to_feet`.

Sie ist unveränderlich.

### `LandingGearFootPositions2D`

`LandingGearFootPositions2D` beschreibt die Weltpositionen beider Fußpunkte:

- `left_x`,
- `left_z`,
- `right_x`,
- `right_z`.

Zusätzlich liefert:

```python
positions.lowest_z
```

die niedrigere der beiden Fußhöhen.

Diese Information wird später von der Touchdown-Erkennung verwendet werden können.

## Verantwortungsgrenze

Die Geometrieschicht entscheidet ausdrücklich nicht, ob ein Bodenkontakt stattgefunden hat.

Sie kennt keine:

- Bodenhöhe,
- Kontaktkräfte,
- Reibung,
- Federung,
- Landekriterien,
- Simulationsabbruchlogik.

Ihre Aufgabe ist ausschließlich:

\[
(\text{Geometrie},HopperState2D)
\rightarrow
\text{Fußpositionen im World Frame}.
\]

Die spätere Touchdown Detection kann darauf aufbauen:

```text
LandingGearGeometry2D
        +
HopperState2D
        ↓
LandingGearFootPositions2D
        ↓
Touchdown Detection
        ↓
Landing Evaluation
```

## Noch keine Hardwareentscheidung

Die Parameter der minimalen Landegeometrie sind zunächst Simulationsparameter.

Beispielwerte wie:

\[
b=0.6\,\mathrm m
\]

oder:

\[
h=0.8\,\mathrm m
\]

stellen keine bereits getroffene Hardwareentscheidung dar.

Sie dienen zunächst dazu, die Geometrie- und Landelogik entwickeln und testen zu können.

Spätere Parameterstudien können genau diese Größen variieren.

## Noch nicht enthalten

Die erste Version modelliert noch keine:

- reale Rumpfkontur,
- Beinlängen oder Beingelenke,
- Fußgröße,
- Federung,
- Kontaktkräfte,
- Reibung,
- elastischen Stoß,
- Kippen nach dem Bodenkontakt,
- strukturelle Belastungen.

Diese Effekte werden erst ergänzt, wenn der erste geometrisch sinnvolle Touchdown-Test funktioniert.
