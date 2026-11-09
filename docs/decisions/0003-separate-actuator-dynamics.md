# ADR 0003: Aktuatordynamik getrennt von Starrkörperdynamik

## Status

Akzeptiert.

## Entscheidung

Die Starrkörperdynamik erhält den tatsächlich wirkenden Schub \(T\) und den tatsächlich erreichten Gimbalwinkel \(\delta\). Aktuatorlimits, Rate-Limits, Verzögerungen und Commands werden außerhalb von `Hopper2DDynamics` modelliert.

## Begründung

Dadurch bleibt die Newton-Euler-Dynamik mathematisch transparent und kann unabhängig von der jeweils verwendeten Hardware wiederverwendet und getestet werden.

Ein späteres ideales oder reales Aktuatormodell kann ausgetauscht werden, ohne die Starrkörpergleichungen zu ändern.
