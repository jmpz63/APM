# The Website Bible — FORGE ZERO Studios Canonical Build Standard

> The single source of truth for every website FORGE ZERO Studios (or Cascade)
> builds. Indexed into the forge1 RAG. If a build decision isn't covered here,
> add it here first, then build.

---

## 1. The 5 Major Website Archetypes

Every client site is one of these (or a hybrid). Identify the archetype FIRST —
it dictates structure, CTA, and which management modules ship with it.

| # | Archetype | Goal | Primary CTA | Killer Feature | Management Modules |
|---|-----------|------|-------------|----------------|--------------------|
| 1 | **Local Service / Brochure** | Phone rings, forms filled | "Book Now" / "Get a Quote" | Social proof (reviews) + instant booking | Scheduling, CRM, invoicing |
| 2 | **E-Commerce / Store** | Sell products | "Add to Cart" | Frictionless checkout | Inventory, orders, payments |
| 3 | **Portfolio / Showcase** | Prove capability | "Hire Me" / "See Work" | Visual gallery, case studies | Lead inbox, project gallery CMS |
| 4 | **Content / Authority (Blog, Media)** | Traffic + trust | "Subscribe" | Search-optimized articles | CMS, newsletter, analytics |
| 5 | **Booking / Lead-Gen Funnel** | Capture appointments | "Schedule" | Calendar with live availability | Scheduling, reminders, payments |

**House cleaning = Archetype 1 + 5 hybrid**: local service brochure with a
booking funnel and a full back office.

## 2. Non-Negotiable Structure (public site)

Order matters — this is the conversion spine, top to bottom:

1. **Announcement / live banner** — offers or scrolling social proof. Motion
   must be slow, pausable (hover), and `prefers-reduced-motion` aware.
2. **Sticky nav** — logo left, 4–6 links, one high-contrast CTA button right.
3. **Hero** — one headline (outcome, not service), one subline, one CTA,
   one supporting visual. 5-second test: a stranger must know what you do,
   where, and what to click.
4. **Trust bar** — ratings, years, jobs done, badges. Numbers beat adjectives.
5. **Services** — cards, 3–6, each with price anchor and its own CTA.
6. **How it works** — 3 steps max. Reduces perceived risk.
7. **Reviews / testimonials** — real names, photos, star ratings, recency.
8. **Pricing** — transparent tiers or "from $X". Hiding price loses locals.
9. **Booking / quote form** — ≤ 6 fields. Every extra field costs conversions.
10. **FAQ** — objection handling (insured? pets? supplies? cancellation?).
11. **Footer** — NAP (name, address, phone) consistent with Google Business
    Profile, hours, service area, socials, legal links.

## 3. Design Laws

- **One brand color + one accent + neutrals.** Never more. (Cleaning Queens:
  pink `#ff2d92` on black `#0a0a0c` with white text.)
- **Type scale**: one display face, one body face; sizes on a modular scale
  (1.25 ratio). Line-height 1.5+ body, 1.1 display.
- **Whitespace is a feature.** Sections breathe: 96–160px vertical padding
  desktop, 56–80px mobile.
- **Radius + shadow consistency**: pick one radius (e.g. 16px) and one
  elevation system; use everywhere.
- **Motion**: subtle entrance reveals (IntersectionObserver), micro-hover
  states, marquee for review banners. Never animate layout after load (CLS).
- **Dark themes**: never pure #000 on pure #fff text — use #0a0a0c bg,
  #f5f5f7 text; accent must pass 4.5:1 contrast on the background.
- **Mobile-first**: design at 390px, enhance up. Thumb-reachable CTAs.

## 4. Tech Stack Rules

- **Static-first**: semantic HTML5 + modern CSS (custom properties, grid,
  `clamp()` fluid type, container queries where useful) + vanilla ES modules.
  Zero build step for preview sites; frameworks only when app complexity
  demands it.
- **No jQuery, no CSS frameworks in previews** — hand-rolled design tokens in
  `:root`.
- **State**: `localStorage` store with a versioned seed (fake data pack) for
  demos; swap for API later without touching UI code.
- **Icons**: inline SVG only (no icon-font requests).
- **Images**: real `<img>` with width/height set (no CLS), lazy-loaded below
  the fold; CSS gradients/patterns where photos aren't available yet.
- **Performance budget**: < 100KB HTML+CSS+JS per page (excluding images),
  LCP < 2.5s, zero layout shift, Lighthouse ≥ 90 all categories.

## 5. SEO & Local

- One `<h1>` per page. Title tag: `{Service} in {City} | {Brand}`.
- Meta description ≤ 155 chars with CTA.
- Schema.org JSON-LD: `LocalBusiness` (+ `aggregateRating` when reviews are
  real), `FAQPage` on the FAQ.
- OpenGraph + Twitter cards on every page.
- NAP consistency across site + Google Business Profile.

## 6. Accessibility (WCAG 2.2 AA)

- Contrast ≥ 4.5:1 body, ≥ 3:1 large text/UI.
- Every interactive element keyboard-reachable with visible focus ring.
- `aria-label`s on icon-only buttons; form inputs always have labels.
- Marquees/carousels: pause on hover/focus, honor `prefers-reduced-motion`.
- Skip-to-content link first in DOM.

## 7. Admin Edit Mode (STANDING RULE — every site ships with it)

While an **admin is signed in**, everything on the live site is editable
inline; public visitors see a normal site with zero edit affordances.

- Gate behind admin auth/session.
- Inline edit all content: headings, paragraphs, images, links, sections,
  and theme colors where feasible.
- **Persist** edits (server write-back in production; versioned
  `localStorage` overlay for static demos) — never ephemeral DOM-only.
- Edit UI: floating toolbar (Save / Discard / Exit), dashed outlines on
  editable regions, dirty-state indicator.

## 8. Management Suite (the "software" behind the site)

Ship as a separate `/admin` app sharing the same data store. Modules:

| Module | Must-haves |
|--------|-----------|
| **Sign-in** | Session token, role (admin/staff), logout, lockout message |
| **Dashboard** | Today's jobs, revenue MTD, pending payments, new leads, star rating |
| **Scheduling** | Week calendar, drag-status jobs, assign staff, recurring jobs (weekly/bi-weekly/monthly), conflicts flagged |
| **Clients (CRM)** | Profile, service history, notes, lifetime value, recurring plan |
| **Payments** | Invoice list (paid/due/overdue), record payment, totals, Stripe-ready abstraction |
| **Project management** | Kanban: Leads → Scheduled → In Progress → Done; per-job checklist |
| **Reviews** | Moderate which reviews appear on the public banner |
| **Staff** | Roster, assignments, availability |
| **Settings** | Business info, hours, service area, theme, edit-mode entry |

Rules: all writes optimistic + persisted to store; every list filterable;
every money figure formatted with `Intl.NumberFormat`; dates via
`Intl.DateTimeFormat` — no date libraries for demos.

## 9. Fake Data Pack (demo realism standard)

Every preview ships believable seeded data: 10+ reviews (names, dates,
star ratings, neighborhood mentions), 8+ clients, 15+ jobs across
past/today/future, invoices in all states, 3+ staff, realistic pricing.
Names diverse, dates relative to "today" so demos never look stale.

## 10. Launch Checklist

- [ ] Archetype identified, spine sections in order (§2)
- [ ] Design tokens only — no hardcoded rogue colors
- [ ] Mobile 390px pass, desktop 1440px pass
- [ ] Lighthouse ≥ 90 ×4
- [ ] Schema JSON-LD validates
- [ ] Admin edit mode works and persists; invisible logged-out
- [ ] Management suite CRUD round-trips the store
- [ ] Reviews banner: slow, pausable, reduced-motion fallback
- [ ] 404 page, favicon, OG image
- [ ] All fake data flagged for replacement before real launch
