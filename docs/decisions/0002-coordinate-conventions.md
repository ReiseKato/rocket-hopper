# ADR 0002: 2D-Koordinaten- und Winkelkonventionen

## Status

Akzeptiert.

## Entscheidung

Für das erste 2D-Modell gelten:

- \(+x_W\) nach rechts,
- \(+z_W\) nach oben,
- \(+y_W\) in die Bildebene,
- \(\theta=0\) für den aufrechten Hopper,
- \(\theta>0\) für Nase nach rechts,
- \(+z_B\) vom Triebwerk zur Nase,
- \(\delta>0\) für einen Schubvektor in Richtung \(+x_B\).

Daraus folgt für das Gimbalmoment

\[
\tau_y=-lT\sin\delta.
\]

## Konsequenz

Diese Seite und `fast_sim/coordinate_system.md` sind die Referenz für alle späteren Vorzeichenentscheidungen. Änderungen an diesen Konventionen wären API- und Modelländerungen und dürfen nicht beiläufig erfolgen.
