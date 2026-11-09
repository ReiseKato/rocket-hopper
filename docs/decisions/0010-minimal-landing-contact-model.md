# ADR 0010: Minimales Kontaktmodell für 2D-Landung

## Status

Akzeptiert

## Kontext

Eine 2D-Landung gilt geometrisch als abgeschlossen, wenn beide Fahrwerksfüße den Boden erreicht haben.

Dies kann auf zwei Arten geschehen:

1. sequentiell: ein Fuß erreicht zuerst den Boden, danach der zweite,
2. simultan: beide Füße erreichen den Boden gleichzeitig.

Nach einem Kontakt kann die freie Starrkörperdynamik nicht unverändert weiterverwendet werden.

Ein vollständiges Feder-Dämpfer-Kontaktmodell würde jedoch noch unbekannte Hardwareparameter wie Steifigkeit, Dämpfung, Reibung und reale Beincompliance benötigen.

## Entscheidung

Landing V0 verwendet vollständig unelastische, haftende Punktkontakte ohne Schlupf.

### Sequentieller Kontakt

Beim ersten Einzelkontakt wird der Drehimpuls um den kontaktierenden Fuß erhalten.

Der Fuß bleibt anschließend als fester Punkt am Boden.

Das Fahrzeug besitzt danach nur noch einen rotatorischen Freiheitsgrad um diesen Kontaktpunkt.

Der vorhandene `TouchdownDetector2D` erkennt später den zweiten Fußkontakt.

### Simultaner Kontakt

Erreichen beide Füße geometrisch gleichzeitig den Boden, werden durch die beiden getrennten haftenden Kontaktpunkte alle planaren Starrkörpergeschwindigkeiten blockiert.

Nach dem Impact gilt:

\[
v_x^+=v_z^+=\omega^+=0.
\]

Die gesamte kinetische Energie unmittelbar vor dem Stoß wird im idealisierten Modell dissipiert.

Eine Aufteilung dieser Energie auf die beiden Beine wird nicht vorgenommen, da sie ohne Compliance-Modell nicht eindeutig bestimmbar ist.

Die Geschwindigkeiten beider Füße unmittelbar vor dem Impact werden jedoch getrennt aufgezeichnet.

### Diskrete BOTH-Erkennung

`TouchdownContact2D.BOTH` allein gilt nicht automatisch als simultaner Kontakt.

Bei diskreten Zeitschritten können beide Füße am Ende eines Schritts unterhalb des Bodens liegen, obwohl einer physikalisch früher aufgesetzt hätte.

`SimultaneousContactImpactTransition2D` fordert deshalb zusätzlich, dass die Fußhöhen innerhalb einer konfigurierbaren geometrischen Toleranz übereinstimmen.

Ist dies nicht erfüllt, muss die spätere Orchestrierung den früheren Einzelkontakt auflösen.

## Konsequenzen

### Positive Konsequenzen

- Sequentielle und simultane Landungen sind beide explizit modelliert.
- Ein simultaner Kontakt benötigt keine künstliche Reihenfolge der Beine.
- Der zweite Fußkontakt einer sequentiellen Landung entsteht aus einer physikalisch eingeschränkten Bewegung.
- Es werden keine unbekannten Feder- oder Dämpferparameter benötigt.
- Impact-Geschwindigkeiten können für jedes kontaktierende Bein bestimmt werden.
- Dissipierte Fahrzeugenergie kann für sequentielle Einzelstöße und den simultanen Gesamtstoß bestimmt werden.
- Die spätere Orchestrierung besitzt zwei klar definierte physikalische Kontaktpfade.

### Negative Konsequenzen

- Kontakte dürfen im Modell nicht rutschen oder wieder abheben.
- Reale Kontaktkräfte und Lastspitzen können noch nicht berechnet werden.
- Beim simultanen Kontakt kann die Energie nicht physikalisch eindeutig auf beide Beine verteilt werden.
- Die genaue zeitliche Reihenfolge zweier Kontakte innerhalb eines groben Simulationsschritts benötigt später eine Event-Lokalisierung.
