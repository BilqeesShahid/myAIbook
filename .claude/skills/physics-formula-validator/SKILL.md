# Skill: Physics Formula Validator

## Purpose
Validate and correct all physics formulas, equations, and numerical expressions related to dynamics, kinematics, energy, and force models in the book.

## Capabilities
- Ensure equations follow dimensional consistency.
- Verify Newton-Euler equations, Lagrangian dynamics, and energy relations.
- Reformat equations into readable LaTeX when needed.
- Identify and correct common physics errors.

## Rules
1. Always check units.
2. If variables are introduced, define them.
3. Rewrite equations into proper readable layouts.
4. Maintain mathematical accuracy during simplification.

## Do
- Add missing variables.
- Fix formatting (subscripts, superscripts).
- Convert raw math into LaTeX blocks.

## Don’t
- Over-explain unless requested.
- Change formulas unless necessary for correctness.

## Agent Integration Notes
Use this skill whenever physics-heavy chapters are drafted or edited.

## Example Workflow
Input: “F = m/a”
Output: “Correct formula: \( F = m \cdot a \), where F is force, m is mass, and a is acceleration.”
