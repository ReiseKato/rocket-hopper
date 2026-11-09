# Landing Evaluation

## Zweck

Die Landing Evaluation bewertet eine geometrisch abgeschlossene, sequentielle 2D-Landung.

Dabei werden drei Begriffe getrennt:

```text
First Contact
    =
erster Fuß erreicht den Boden

Landing Completed
    =
zweiter Fuß erreicht den Boden

Landing Successful
    =
Landing Completed
+
alle konfigurierten Erfolgskriterien erfüllt
```

Die Evaluation beeinflusst weder Guidance noch Controller oder Kontaktdynamik.

Sie wertet ausschließlich bereits entstandene Zustände und Kontaktübergänge aus.

## Erfolgskriterien

`LandingSuccessCriteria2D` enthält die konfigurierbaren Grenzwerte:

- maximaler horizontaler Positionsfehler,
- maximale normale Geschwindigkeit beim ersten Kontakt,
- maximale tangentiale Geschwindigkeit beim ersten Kontakt,
- maximale normale Geschwindigkeit beim zweiten Kontakt,
- maximale tangentiale Geschwindigkeit beim zweiten Kontakt,
- maximaler Fahrzeugwinkel beim zweiten Kontakt,
- maximale Winkelgeschwindigkeit beim zweiten Kontakt.

Die Kriterien sind Simulationsanforderungen.

Sie stellen noch keine validierten Hardware- oder Sicherheitsgrenzen dar.

## Fußgeschwindigkeit

Für den Vektor vom Schwerpunkt zum betrachteten Fuß:

\[
r=
\begin{bmatrix}
r_x\\
r_z
\end{bmatrix}
\]

gilt:

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

Für einen horizontalen Boden wird die normale Aufsetzgeschwindigkeit als positiver Betrag in Bodenrichtung definiert:

\[
v_n
=
\max(0,-v_{z,\mathrm{foot}}).
\]

Die tangentiale Geschwindigkeit lautet:

\[
v_t
=
|v_{x,\mathrm{foot}}|.
\]

Diese Größen werden für ersten und zweiten Fußkontakt getrennt gespeichert.

## `LandingContactMetrics2D`

Für jeden sequentiellen Fußkontakt werden aufgezeichnet:

- kontaktierender Fuß,
- Kontaktzeit,
- normale Aufsetzgeschwindigkeit,
- tangentiale Aufsetzgeschwindigkeit,
- Fahrzeugwinkel,
- Winkelgeschwindigkeit,
- Schwerpunktgeschwindigkeit,
- kinetische Energie vor dem Stoß,
- kinetische Energie nach dem Stoß,
- dissipierte Energie des idealisierten Stoßes.

Die Energiegrößen beziehen sich auf das gewählte Kontaktmodell.

Sie entsprechen nicht automatisch der realen strukturellen Energieaufnahme eines einzelnen Fahrwerkbeins.

## Erster Kontakt

Die Metriken des ersten Kontakts werden aus:

- dem Starrkörperzustand unmittelbar vor dem Stoß,
- `FirstContactImpactResult2D`

gebildet.

Die bereits von der Impact Transition berechneten Energien werden wiederverwendet.

## Zweiter Kontakt

Die zweite Kontaktbewertung darf nur erfolgen, wenn der Touchdown Detector:

```python
TouchdownContact2D.BOTH
```

liefert.

Der zweite Fuß ist der jeweils andere Fuß des ersten Kontakts.

Seine Aufsetzgeschwindigkeit wird aus dem Starrkörperzustand unmittelbar vor dem zweiten Kontakt berechnet.

Für Landing V0 wird der zweite Kontakt ebenfalls vollständig unelastisch betrachtet.

Nach dem zweiten Kontakt gilt:

\[
v_x^+=0,
\qquad
v_z^+=0,
\qquad
\omega^+=0.
\]

Damit ist:

\[
E_2^+=0
\]

und:

\[
E_{\mathrm{diss},2}
=
E_2^-.
\]

## Gesamtenergie

Die gesamte im idealisierten Kontaktmodell dissipierte Stoßenergie lautet:

\[
E_{\mathrm{diss,total}}
=
E_{\mathrm{diss},1}
+
E_{\mathrm{diss},2}.
\]

Diese Größe wird gemessen, aber noch nicht als Erfolgskriterium verwendet.

Ohne reale Fahrwerks-, Material- und Strukturdaten existiert noch kein physikalisch begründeter zulässiger Grenzwert.

## Horizontale Zielabweichung

Beim zweiten Kontakt wird der signierte Fehler gespeichert:

\[
e_x
=
p_x-p_{x,\mathrm{target}}.
\]

Für das Erfolgskriterium wird sein Betrag verwendet:

\[
|e_x|
\le
e_{x,\max}.
\]

## Fehlgründe

`LandingFailureReason2D` macht eine fehlgeschlagene Landung auswertbar.

Mögliche Gründe sind:

```text
HORIZONTAL_POSITION_ERROR
FIRST_CONTACT_NORMAL_SPEED
FIRST_CONTACT_TANGENTIAL_SPEED
SECOND_CONTACT_NORMAL_SPEED
SECOND_CONTACT_TANGENTIAL_SPEED
ATTITUDE_ANGLE
ANGULAR_SPEED
```

Mehrere Gründe können gleichzeitig vorliegen.

Das ist insbesondere für spätere Parameterstudien wichtig.

## `LandingEvaluation2D`

Nach dem zweiten Kontakt enthält die finale Evaluation:

- `completed`,
- `successful`,
- Metriken des ersten Kontakts,
- Metriken des zweiten Kontakts,
- horizontalen Positionsfehler,
- Zeit zwischen beiden Kontakten,
- gesamte dissipierte Stoßenergie,
- alle verletzten Erfolgskriterien.

Eine `LandingEvaluation2D` repräsentiert in der aktuellen Version nur eine bereits geometrisch abgeschlossene Landung.

Daher gilt immer:

```python
evaluation.completed is True
```

und:

```python
evaluation.successful
```

ist genau dann wahr, wenn `failed_criteria` leer ist.

## Keine künstliche Härtekennzahl

Die erste Version berechnet bewusst keinen dimensionslosen Landing-Hardness-Score.

Stattdessen bleiben die physikalischen Messgrößen sichtbar:

\[
v_{n,1},
\quad
v_{t,1},
\quad
E_{\mathrm{diss},1},
\]

\[
v_{n,2},
\quad
v_{t,2},
\quad
E_{\mathrm{diss},2}.
\]

Damit kann später auf Basis realer Hardwaredaten entschieden werden, welche Größen tatsächlich kritisch sind.

## Direkter gleichzeitiger Zweifußkontakt

Die aktuelle Evaluation beschreibt die sequentielle Kontaktfolge:

```text
ein Fuß
    ↓
Single-Contact-Phase
    ↓
zweiter Fuß
```

Ein exakt gleichzeitiger Erstkontakt beider Füße besitzt keine physikalisch eindeutige Reihenfolge von „erstem“ und „zweitem“ Bein.

Eine willkürliche Aufteilung der dissipierten Energie auf beide Beine wäre deshalb nicht sinnvoll.

Dieser Sonderfall wird bei der Landing-Orchestrierung separat behandelt beziehungsweise kann später durch ein eigenes simultanes Zweikontaktmodell ergänzt werden.
