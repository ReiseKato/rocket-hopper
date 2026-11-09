# ADR 0004: Numerische Zustandsdarstellung

## Status

Akzeptiert

## Kontext

Der Simulator verwendet semantische, objektorientierte Datentypen wie `HopperState2D`, damit die physikalische Bedeutung der Zustandsgrößen im Code explizit bleibt.

Numerische Integrationsverfahren wie Euler oder RK4 benötigen jedoch Vektoroperationen.

Ein RK4-Integrator muss beispielsweise Zwischenzustände der Form

\[
x_{\mathrm{mid}}
=
x_k
+
\frac{\Delta t}{2}k_1
\]

berechnen.

Würden im gesamten Simulator ausschließlich rohe NumPy-Vektoren verwendet, wäre die fachliche Bedeutung der einzelnen Zustandsgrößen an den Komponentenschnittstellen weniger eindeutig.

Eine vollständige Implementierung mathematischer Vektoroperationen direkt in den Domänenklassen würde dagegen Zustandsobjekte unnötig stark mit numerischen Integrationsverfahren koppeln.

## Entscheidung

Die kanonische öffentliche Zustandsdarstellung bleibt `HopperState2D`.

Numerische Algorithmen dürfen Zustände intern explizit in NumPy-Vektoren umwandeln.

Dafür werden folgende Konvertierungsschnittstellen verwendet:

- `HopperState2D.to_vector()`
- `HopperState2D.from_vector()`
- `HopperStateDerivative2D.to_vector()`

Die kanonische Reihenfolge des 2D-Zustandsvektors lautet:

\[
x=
\begin{bmatrix}
p_x &
p_z &
v_x &
v_z &
\theta &
\omega
\end{bmatrix}^{T}.
\]

Integratoren dürfen NumPy-Vektoren intern für mathematische Operationen verwenden, müssen an ihrer äußeren Schnittstelle jedoch semantische Zustandsobjekte entgegennehmen und zurückgeben.

Die Domänenklassen erhalten keine allgemeinen arithmetischen Operatoren für Integrationsoperationen.

Insbesondere werden keine Operatoren wie

```python
state + derivative
```

oder

```python
state * scalar
```

eingeführt.

Die mathematische Kombination von Zustand, Zustandsableitung und Zeitschritt bleibt Verantwortung des Integrators.

## Konsequenzen

### Positive Konsequenzen

- Die physikalische Bedeutung der Zustandsgrößen bleibt an den Komponentenschnittstellen explizit.
- Numerische Integratoren können normale Vektoroperationen verwenden.
- Integratoren müssen die einzelnen Hopper-Zustandsfelder nicht direkt kennen.
- Die Architektur lässt sich später gut auf 3D-Zustände erweitern.
- NumPy und andere numerische Bibliotheken können verwendet werden, ohne rohe Arrays zum fachlichen Zustandsmodell zu machen.
- Zustands- und Integrationslogik bleiben klar voneinander getrennt.

### Negative Konsequenzen

- Zwischen Zustandsobjekten und NumPy-Vektoren sind explizite Konvertierungen notwendig.
- Die Reihenfolge der Elemente im Zustandsvektor wird Teil der Schnittstellendefinition und muss dokumentiert und getestet werden.
- RK4-Zwischenzustände müssen vor der Auswertung der Dynamik wieder in einen HopperState2D umgewandelt werden.
- Die Konvertierungen erzeugen einen kleinen zusätzlichen Implementierungsaufwand.

## Begründung

Die gewählte Lösung verbindet eine physikalisch nachvollziehbare OOP-Schnittstelle mit einer für numerische Algorithmen geeigneten Vektordarstellung.

Sie verhindert, dass interne numerische Details die öffentliche Architektur des Simulators bestimmen.

Die zentrale Regel lautet:

> NumPy-Vektoren sind eine numerische Repräsentation, nicht das fachliche Zustandsmodell.