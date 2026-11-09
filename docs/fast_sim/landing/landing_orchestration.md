# Landing-Orchestrierung

## Zweck

Die Landing-Orchestrierung verbindet die bisher getrennt entwickelten Komponenten zu einem vollständigen 2D-Landeablauf.

Sie koordiniert:

- Landing Guidance,
- horizontale Positionsregelung,
- Lageregelung,
- Höhenregelung,
- physische Actuation,
- freie Starrkörperdynamik,
- Touchdown Detection,
- sequentielle und simultane Impact-Modelle,
- Single-Contact-Dynamik,
- Landing Evaluation.

Die Orchestrierung enthält keine neuen Bewegungsgleichungen und keine neue Reglerlogik.

## Physikalische Dynamikmodi

Die Orchestrierung unterscheidet ausschließlich den aktuell gültigen physikalischen Modus:

```text
FREE_FLIGHT
    ↓
SINGLE_CONTACT
    ↓
COMPLETED
```

Ein simultaner Zweifußkontakt wechselt direkt von:

```text
FREE_FLIGHT
    ↓
COMPLETED
```

Diese Modi beschreiben ausschließlich die aktuell gültige Physik.

## Abgrenzung zu späteren Mission States

Spätere Zustände wie:

```text
START
    ↓
TARGET_POSITION_REACHED
    ↓
DESCENT
    ↓
LANDING_APPROACH
    ↓
LANDING
```

beschreiben eine andere Ebene.

Sie beantworten:

> Welche Aufgabe soll das Fahrzeug gerade ausführen?

Die Landing-Dynamikmodi beantworten dagegen:

> Welches physikalische Bewegungsmodell gilt gerade?

Die beiden State Machines sollen deshalb nicht vermischt werden.

Eine spätere Mission-State-Machine kann oberhalb von Guidance und Control liegen und beispielsweise zwischen Position Hold, Sinkflug und Landing Guidance wechseln.

## Gesamtstruktur

```text
LandingGuidance2D
        ↓
Controller Stack
        ↓
HopperActuationCommand2D
        ↓
FREE_FLIGHT / HopperSystem2D
        ↓
Touchdown Detection
        │
        ├── echter simultaner Kontakt
        │         ↓
        │ SimultaneousContactImpactTransition2D
        │         ↓
        │ SimultaneousLandingEvaluation2D
        │         ↓
        │      COMPLETED
        │
        └── erster Einzelkontakt
                  ↓
          FirstContactImpactTransition2D
                  ↓
             SINGLE_CONTACT
                  ↓
          SingleContactDynamics2D
                  ↓
             zweiter Fuß
                  ↓
           LandingEvaluation2D
                  ↓
              COMPLETED
```

## Freier Flug

Im Modus:

```python
LandingDynamicsMode2D.FREE_FLIGHT
```

wird weiterhin `HopperSystem2D` verwendet.

Guidance und Controller erzeugen zunächst:

```text
HorizontalPositionReference2D
        ↓
HorizontalPositionController2D
        ↓
AttitudeReference2D
        ↓
AttitudeController2D
        ↓
delta_cmd

AltitudeReference2D
        ↓
AltitudeController2D
        ↓
thrust_cmd
```

Beide Commands werden zu:

```python
HopperActuationCommand2D
```

kombiniert und an `HopperSystem2D` übergeben.

## Simultaner Zweifußkontakt

Liefert die Touchdown Detection:

```python
TouchdownContact2D.BOTH
```

wird zusätzlich geprüft, ob die beiden Fußhöhen innerhalb der vom simultanen Impact-Modell definierten Toleranz übereinstimmen:

\[
|z_L-z_R|
\leq
\varepsilon_z.
\]

Nur dann wird der Kontakt als tatsächlich simultan behandelt.

Der Ablauf ist:

```text
BOTH
  +
geometrisch simultan
        ↓
SimultaneousContactImpactTransition2D
        ↓
SimultaneousLandingEvaluation2D
        ↓
COMPLETED
```

Damit ist ein simultaner Kontakt ein vollständig gültiger Landeabschluss.

## Diskretes `BOTH` ist nicht automatisch simultan

Bei einem endlichen Zeitschritt kann ein Fuß früher den Boden kreuzen und der zweite noch innerhalb desselben Schritts folgen.

Der gespeicherte Endzustand kann dann bereits:

```python
TouchdownContact2D.BOTH
```

liefern, obwohl die physikalische Kontaktfolge sequentiell war.

Die Orchestrierung betrachtet deshalb zusätzlich die Bodenfreiheiten vor und nach dem Schritt.

Für einen Fuß wird der ungefähre Kreuzungsanteil des Zeitschritts bestimmt:

\[
\alpha
=
\frac{c_{\mathrm{previous}}}{
c_{\mathrm{previous}}-c_{\mathrm{current}}
}.
\]

Der kleinere Wert von \(\alpha\) bezeichnet den früheren Bodenkontakt.

Dieser Fuß wird als erster Einzelkontakt verwendet.

Die Impact Transition wird in V0 weiterhin auf den aktuellen diskreten Starrkörperzustand angewendet.

Damit wird die Kontaktreihenfolge besser aufgelöst, ohne bereits eine vollständige Event-Lokalisierung einzuführen.

## Grenze der diskreten Kontaktauflösung

Die aktuelle Orchestrierung lokalisiert den exakten Impact-Zeitpunkt innerhalb des freien Flugschritts noch nicht.

Das bedeutet:

- die Kontaktreihenfolge wird innerhalb eines Schritts abgeschätzt,
- der Impact-Zustand bleibt der am Schrittende berechnete Zustand,
- der erste Fuß wird anschließend auf die Bodenebene projiziert.

Eine spätere Event-Lokalisierung kann:

- den exakten Nulldurchgang bestimmen,
- den Free-Flight-Schritt am Impact-Zeitpunkt beenden,
- den verbleibenden Teil des Zeitschritts bereits mit der Kontaktdynamik berechnen.

Für Landing V0 ist diese Genauigkeit bewusst noch nicht erforderlich.

## Sequentieller erster Kontakt

Wird genau ein erster Fuß bestimmt, wird:

```python
FirstContactImpactTransition2D
```

ausgeführt.

Dabei entstehen:

- der feste Kontaktpunkt,
- `SingleContactState2D`,
- der post-impact `HopperState2D`,
- die Impact-Energie,
- die First-Contact-Metriken.

Danach gilt:

```python
LandingDynamicsMode2D.SINGLE_CONTACT
```

## Single-Contact-Dynamik

Nach dem ersten Kontakt wird `HopperSystem2D` nicht mehr für die Starrkörperbewegung verwendet.

Die freie Starrkörperdynamik ist mit einem fest haftenden Fuß nicht mehr gültig.

Stattdessen wird:

\[
[\theta,\omega]^T
\]

mit `SingleContactDynamics2D` integriert.

`IntegratedSingleContactStepper2D` verwendet dafür weiterhin die allgemeine `Integrator`-Schnittstelle.

Der numerische Integrator bleibt damit auch in der Kontaktphase austauschbar.

## Aktuatorzustand nach dem ersten Kontakt

Schub- und Gimbalaktuator bleiben nach dem ersten Kontakt aktiv.

Insbesondere darf der Gimbalzustand beim Moduswechsel nicht zurückgesetzt werden.

Deshalb erhält die Orchestrierung:

```python
PhysicalActuationProvider2D
```

mit:

```python
(command, dt) -> HopperActuation2D
```

Dieser Provider muss denselben Aktuatorzustand weiterführen, den `HopperSystem2D` im freien Flug verwendet hat.

Die Landing-Orchestrierung kennt dadurch keine konkreten internen Aktuatorklassen.

## Zweiter Fußkontakt

Während `SINGLE_CONTACT` ist der erste Fuß durch die Zwangsbedingung bereits fest am Boden.

Deshalb muss für den Abschluss nur geprüft werden, ob der gegenüberliegende Fuß:

\[
z_{\mathrm{second}}
\leq
z_{\mathrm{ground}}
\]

erreicht.

Sobald dies geschieht, wird für die Evaluation ein `BOTH`-Kontaktzustand erzeugt.

`LandingEvaluator2D` berechnet dann:

- zweite normale Aufsetzgeschwindigkeit,
- zweite tangentiale Aufsetzgeschwindigkeit,
- zweite dissipierte Impact-Energie,
- Zeit zwischen den Kontakten,
- horizontalen Positionsfehler,
- Success Criteria,
- Failure Reasons.

## Abschlusszustand der sequentiellen Landung

Bei einem diskreten zweiten Kontakt kann die Fahrzeuglage geringfügig über die geometrisch exakte Zweifußlage hinauslaufen.

Nach der Evaluation wird der endgültige Zustand deshalb auf die stationäre Zweifußlage projiziert:

\[
\theta=0,
\]

\[
\omega=0,
\]

\[
v_x=0,
\]

\[
v_z=0.
\]

Die Schwerpunktposition ergibt sich aus dem festen ersten Kontaktpunkt und der Fahrwerksgeometrie.

Auch diese Projektion kann später durch exakte Event-Lokalisierung präzisiert werden.

## `LandingContactSequence2D`

Ein abgeschlossener Run dokumentiert zusätzlich die Kontaktart:

```python
LandingContactSequence2D.SEQUENTIAL
LandingContactSequence2D.SIMULTANEOUS
```

Damit können spätere Auswertungen und Parameterstudien beide Landetypen getrennt analysieren.

## `LandingRunResult2D`

Das Ergebnis enthält:

- Zeitpunkte,
- Starrkörperzustände,
- physikalische Dynamikmodi,
- Guidance References,
- Commands,
- physikalische Actuations,
- First-Contact-Metriken bei sequentieller Landung,
- finale Evaluation,
- Kontaktfolge,
- Abbruchgrund.

Für \(N\) ausgeführte Schritte gilt:

```text
times                  N + 1
rigid_body_states      N + 1
modes                  N + 1

guidance_references    N
commands               N
physical_actuations    N
```

Damit bleibt die bekannte Ergebnissemantik des Fast Simulators erhalten.

## Abbruchgründe

Die erste Version unterscheidet:

```python
LandingTerminationReason2D.COMPLETED
LandingTerminationReason2D.MAX_STEPS_REACHED
```

`COMPLETED` gilt sowohl für eine erfolgreiche sequentielle als auch für eine erfolgreiche oder fehlgeschlagene simultane Landung.

Ob die Landung die Anforderungen erfüllt, steht separat in:

```python
result.successful
```

beziehungsweise:

```python
result.evaluation.failed_criteria
```

## One-Shot-Semantik

`LandingOrchestrator2D` ist bewusst nur einmal ausführbar.

Nach einem sequentiellen ersten Kontakt wird die freie Starrkörperbewegung von `HopperSystem2D` verlassen.

Dessen interner Starrkörperzustand entspricht danach nicht mehr der tatsächlichen Zwangsbewegung.

Für jeden neuen Landing Run muss deshalb eine neue System- und Orchestratorinstanz erzeugt werden.

Das passt ebenfalls zu späteren Parameterstudien, bei denen jeder Versuch reproduzierbar aus einem frischen Anfangszustand gestartet wird.

## Verantwortungsgrenze

Die Landing-Orchestrierung entscheidet:

- welcher physikalische Dynamikmodus aktiv ist,
- welcher Kontaktpfad verwendet wird,
- wann Guidance und Controller ausgewertet werden,
- wann zwischen freiem Flug und Kontaktdynamik gewechselt wird,
- wann die Landing Evaluation ausgeführt wird,
- wann der Run beendet wird.

Sie entscheidet nicht:

- welche Missionsphase aktiv ist,
- welche Mission als Nächstes ausgeführt wird,
- wie Guidance intern ihre Trajektorie erzeugt,
- wie Controller intern arbeiten,
- wie reale elastische Kontaktkräfte entstehen.

Damit bleibt sie eine reine physikalische Ablauf- und Modusorchestrierung.
