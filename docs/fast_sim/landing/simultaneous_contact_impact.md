# Simultaner Zweifuß-Impact

## Zweck

Eine 2D-Landung kann geometrisch auch so erfolgen, dass beide Fahrwerksfüße gleichzeitig den Boden erreichen.

Dieser Fall darf nicht künstlich in einen ersten und einen zweiten Fußkontakt zerlegt werden.

Dafür existiert `SimultaneousContactImpactTransition2D`.

## Kontaktbedingung

Der `TouchdownDetector2D` kann:

```python
TouchdownContact2D.BOTH
```

melden.

In einer diskreten Simulation reicht dieses Ergebnis allein jedoch nicht aus, um einen wirklich simultanen Kontakt nachzuweisen.

Bei einem großen Zeitschritt kann beispielsweise zuerst der rechte Fuß den Boden überqueren und innerhalb desselben Schritts auch der linke Fuß.

Der gespeicherte Endzustand besitzt dann zwar beide Füße auf oder unterhalb der Bodenebene, physikalisch war der Kontakt aber sequentiell.

Deshalb prüft das simultane Impact-Modell zusätzlich:

\[
|z_L-z_R|
\leq
\varepsilon_z.
\]

`max_foot_height_difference` beschreibt diese geometrische Toleranz.

Ist die Differenz größer, wird der simultane Übergang abgelehnt.

Eine spätere Landing-Orchestrierung muss dann den früheren Einzelkontakt innerhalb des Zeitschritts auflösen.

## Stoßmodell

Für den simultanen Zweifußkontakt gelten dieselben grundsätzlichen V0-Annahmen wie für den Einzelkontakt:

- vollständig unelastisch,
- haftend,
- kein Schlupf,
- keine elastische Rückfederung.

Da zwei getrennte Punkte des starren 2D-Körpers gleichzeitig am Boden haften, werden alle drei planaren Geschwindigkeitsfreiheitsgrade blockiert.

Nach dem Stoß gilt:

\[
\boxed{
v_x^+=0
}
\]

\[
\boxed{
v_z^+=0
}
\]

\[
\boxed{
\omega^+=0
}.
\]

Der Zustand ist damit unmittelbar nach dem Impact stationär.

## Energie

Vor dem Stoß besitzt das Fahrzeug:

\[
E^-
=
\frac12m(v_x^2+v_z^2)
+
\frac12I_y\omega^2.
\]

Nach dem vollständig unelastischen Zweifußstoß gilt:

\[
E^+=0.
\]

Damit folgt:

\[
\boxed{
E_{\mathrm{diss}}
=
E^-.
}
\]

Diese Größe beschreibt die im idealisierten Kontaktmodell dissipierte kinetische Energie des gesamten Fahrzeugs.

## Keine Aufteilung auf die Beine

Für den simultanen Kontakt wird die dissipierte Energie bewusst nicht in:

\[
E_L
\]

und:

\[
E_R
\]

zerlegt.

Zwei haftende Kontaktpunkte erzeugen mehrere Impulskomponenten.

Ohne zusätzliche Annahmen über:

- Beinsteifigkeit,
- Dämpfung,
- Compliance,
- Kontaktverformung

ist die physikalische Impuls- beziehungsweise Energieverteilung zwischen linkem und rechtem Bein nicht eindeutig.

Eine mathematisch willkürliche Aufteilung würde eine Genauigkeit vortäuschen, die das Modell nicht besitzt.

## Fußgeschwindigkeiten

Die Geschwindigkeit jedes Fußes unmittelbar vor dem Stoß kann trotzdem separat berechnet werden.

Für einen Fuß mit dem Vektor:

\[
r=
\begin{bmatrix}
r_x\\
r_z
\end{bmatrix}
\]

vom Schwerpunkt zum Fuß gilt:

\[
v_{x,\mathrm{foot}}
=
v_x+\omega r_z
\]

und:

\[
v_{z,\mathrm{foot}}
=
v_z-\omega r_x.
\]

Damit stehen für beide Beine getrennt zur Verfügung:

- normale Aufsetzgeschwindigkeit,
- tangentiale Aufsetzgeschwindigkeit.

Diese Größen sind für die Landing Evaluation direkt nutzbar.

## Projektion auf den Boden

Durch die diskrete Integration kann ein geometrisch simultaner Zustand geringfügig unterhalb der Bodenebene liegen.

Wenn beide Fußhöhen innerhalb der Simultaneitätstoleranz übereinstimmen, wird der Schwerpunkt rein vertikal verschoben, sodass die mittlere Fußhöhe auf:

\[
z_{\mathrm{ground}}
\]

liegt.

Die Fahrzeuglage wird durch den instantanen Impact nicht verändert.

Für einen echten simultanen Kontakt mit horizontalem Boden muss die Geometrie bereits eine gemeinsame Fußhöhe besitzen.

## Landing Evaluation

Die simultane Evaluation verwendet:

```python
SimultaneousLandingContactMetrics2D
```

und:

```python
SimultaneousLandingEvaluation2D
```

Sie enthält getrennte Geschwindigkeitsmetriken für linken und rechten Fuß, aber nur eine gemeinsame dissipierte Fahrzeugenergie.

`LandingSuccessCriteria2D` kann optional eigene Grenzwerte definieren:

```python
max_simultaneous_contact_normal_speed
max_simultaneous_contact_tangential_speed
```

Werden sie nicht gesetzt, wird konservativ jeweils der strengere Grenzwert aus erstem und zweitem sequentiellem Kontakt verwendet.

## Bedeutung für die spätere Orchestrierung

Die spätere Landing-Orchestrierung besitzt damit zwei gültige Wege:

```text
FREE_FLIGHT
    │
    ├── echter Einzelkontakt
    │       ↓
    │   FirstContactImpactTransition2D
    │       ↓
    │   SINGLE_CONTACT
    │       ↓
    │   zweiter Kontakt
    │
    └── echter simultaner Kontakt
            ↓
        SimultaneousContactImpactTransition2D
            ↓
        COMPLETED
```

Ein bloßes diskretes `BOTH` darf dabei nicht automatisch als simultaner Kontakt interpretiert werden.
