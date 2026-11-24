# Dashboard style: layout, type and components

This is the look of the agent-team dashboard (see `AGENT_TEAM_PLAYBOOK.md`, section 5).

**Colours are not set here.** The user describes the colour theme in the session. Implement it as light and dark tokens, and give the header a theme button that cycles auto, light and dark.

**Reference:** the `<style>` block of `tools/dashboard/static/index.html` in `C:/Users/aomar/Desktop/Robot project`. Take its layout, type and components from there, but not its colours.

**Character:**
- Surfaces are flat and bordered with hairlines. Corners are square, except small chips and pills, which have a 2 px radius.
- Hierarchy comes from borders and surface steps, never shadows.
- A faint 40 px blueprint grid sits behind the page.
- Labels are uppercase and letter-spaced. "Marker" labels follow a 5 px square bullet.
- One accent gradient, used only on accent lines and progress bars.
- Numbers are tabular.

**Type:**
- Text: `'Effra', 'Effra Std', 'IBM Plex Sans Arabic', 'Segoe UI', system-ui, sans-serif`, 14 px / 1.5.
- Code: `'Cascadia Mono', Consolas, monospace`.
- Icons: Material Symbols Outlined (Google Fonts), hidden when the font doesn't load.
- Page titles: 28-40 px, weight 700, uppercase, with an 80 x 2 px accent line under them.
- Panel titles: 15 px, 700, uppercase, tracking .04em.
- Table headers and labels: 10.5 px, 600, uppercase, tracking .12-.14em.

**Components:**
- **Header:** sticky; a 64 px brand row (a 34 px square brand mark and an uppercase wordmark, 19 px, 700, tracking .12em) and 48 px tabs, with a 12 px backdrop blur. The active tab is bold with a 2 px underline; an underline grows on hover.
- **Live dot:** 7 px and pulsing; its state shows live, stale or down.
- **KPI tiles:** in a 2 px "seam" grid (the seams in the hairline colour), with 1, 2, 3 or 6 columns. The figure is 34 px and 700. Bars are 4 px, plain or split pass/fail.
- **Panels:** 1 px hairline border, 20 px padding, 22 px gaps, page width up to 1360 px.
- **Chips** at 11.5 px, and **status pills** (ok, warn, bad) with a 6 px square bullet.
- **Tables:** hairline rows, highlighted on hover.
- **Agents tab:** kanban columns of at least 170 px with 10 px gaps. Each task gets a colour from a fixed cycle of the theme's colours, used for its card tags and its timeline bars.
- **Motion:** content blurs in on entry (0.55 s, `cubic-bezier(0.22, 1, 0.36, 1)`), and the title underline grows. All motion is off under `prefers-reduced-motion`.
