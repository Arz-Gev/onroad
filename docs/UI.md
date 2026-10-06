# UI rules

Read this before you add or change anything on screen: the HUD, the pause menu, the tuning panel, the
loading screen. The goal is one calm, consistent interface. A new feature fits into the system below.
It does not bring its own look, its own toast or its own colour.

## 1. Look

- **Surfaces**: near-black glass. Use `var(--panel)` (HUD) or `var(--sheet)` (menu) with
  `var(--glass-blur)`, a 1px `var(--line)` border, and radius 8–12 (HUD lengths in `calc(N * var(--u))`).
  No drop shadows on HUD elements. No gradients.
- **Colour**: white text in three steps: `--txt`, `--dim`, `--faint` (plus `--ghost` for tracks and
  outlines). **Two accents, never more**:
  - `--accent` (white): on, selected, primary button, rpm fill.
  - `--hot` (orange-red): warning, active intervention (ABS / TC working, handbrake, wheel slip),
    redline, a value changed from stock.
  No green for "good", no blue, no amber. "Good" is plain white. If something needs attention, it is hot.
  Every colour is a token in `:root` (`src/ui.css`). Canvas code uses the same values (`HOT` in hud.js).
- **Type**: Geist (sans) and Geist Mono (numbers that change: rpm scale, readouts). Weights 400–600.
  Sentence case. Uppercase only for TC / ABS lamp labels. Numbers that update use tabular figures.
- **Icons**: line icons, 24px grid, `stroke="currentColor"`, 1.75–1.8 stroke. Menus use `src/icons.js`;
  HUD lamps are defined in hud.js (`LAMP_ICONS`). Prefer an icon with a `title` to a word, when the icon
  is unambiguous.
- **Motion**: 120–300 ms, ease-out going in and ease-in going out. Nothing bounces or scales. Respect
  `prefers-reduced-motion`.

## 2. No layout shift

Nothing moves because a value changed.
- Fixed slots: lamps are always present (dim when off), info cells have fixed grid columns with
  ellipsis, tabs are one fixed size, settings columns are fixed (`col: 1 | 2` per section).
- Numbers use tabular figures or mono. A value that changes width sits in a box of fixed width.
- Scroll areas use `scrollbar-gutter: stable`.
- Showing or hiding an optional row may move the rows below it, never the ones above or beside it.
- Check with the sweep in DEVNOTES ("UI checks"): click every control and compare positions.

## 3. State or event?

Decide this first for every new piece of information.

- **State**: anything that stays true until it changes (gear, gearbox mode, lights, TC, ABS, handbrake,
  diff / lockers / range, tyre pressure, surface, engine running, sound). It lives in the **cluster**,
  as a lamp (`lampState`) or an info field (`field`). When the player changes it, the field flashes once
  (`flash`). **State is never a notification.** "Low beam" or "ABS off" as a toast is a bug.
- **Event**: something that happened once and is not visible otherwise: a refused action ("Stop to
  change the transfer range"), an error, a result ("0–100 km/h in 4.3 s"), a place you teleported to.
  It goes to the **ticker**.
- **Standing tip**: a hint that holds while a condition holds (engine off, upside down, stuck, sound
  locked). It goes to the ticker as a `hint`, from `HUD.tips()`.

A feature the car doesn't have shows no lamp, row, key hint or menu entry (`view.features`,
`drivetrain.fixedRwd`, a row's `car` filter). If the key is pressed anyway: `noFeature`.

## 4. Notifications (the ticker)

- One line above the cluster, the cluster's width. It's empty by default. A new line slides in from
  below, and the old one slides up and fades. One line at a time: the newest event wins, and a standing
  tip shows when no event is.
- Levels: `info` (white dot), `warn` (hot dot and border), `hint` (hollow dot, dimmer). No other levels.
- **All text lives in `src/messages.js`**, keyed by id. Code calls `hud.notify(id, params)` (or
  `drivetrain.say(id, params)` from the physics, which the HUD picks up). No HTML or text for the
  player is built anywhere else. Params are escaped. Key caps come from `k(bindingId)`, so they follow
  the device.
- Text: one line, at most about 48 characters plus key caps, sentence case, no full stop, `·` between
  the fact and the action ("Stalled · I to restart").
- The same id again updates the line in place with a short nudge. It does not stack.

## 5. Naming

CSS is global. Prefix every class with its component: `tk-` ticker, `tn-` tuning panel, `set-`
settings, `loc-` locations, `ctl-` controls, `cl-` / `rpm-` / `lamp` cluster. Never reuse generic
names like `.info`, `.hint`, `.hints`, `.warn` across components. Two bugs already came from
exactly that.

## 6. Adding something: checklist

1. State or event? (section 3)
2. State: add a lamp or an info cell to the cluster in hud.js, with a fixed slot and a flash on change.
   Make sure it still fits the compact cluster.
3. Event: add an id to `messages.js` and call `hud.notify(id)`.
4. Use colours and sizes from tokens only. Two accents.
5. Hide it on cars that don't have the feature.
6. Run the checks in DEVNOTES ("UI checks") and look at a screenshot at 1280×800 and at a small window.
