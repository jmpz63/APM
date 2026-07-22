/* Cleaning Queens — shared data store (demo: localStorage-backed, seeded fake data)
   Swap CQStore internals for API calls in production without touching UI code. */
(function (global) {
  "use strict";

  const VERSION = 4;
  const KEY = "cq_data_v" + VERSION;
  const SESSION_KEY = "cq_session";

  const day = 86400000;
  const now = new Date();
  const d = (offset, h = 9, m = 0) => {
    const t = new Date(now.getFullYear(), now.getMonth(), now.getDate() + offset, h, m);
    return t.toISOString();
  };

  const SEED = {
    version: VERSION,
    business: {
      name: "The Cleaning Queens KC",
      tagline: "Because your home deserves the royal treatment!",
      phone: "(816) 269-5228",
      phone2: "(319) 215-6438",
      email: "thecleaningqueenskc1@gmail.com",
      address: "Kansas City, KS & Kansas City, MO",
      hours: "Mon–Sat 8:00 AM – 6:00 PM",
      serviceArea: "Kansas City metro — both sides of the state line",
      facebook: "https://www.facebook.com/p/The-Cleaning-Queens-KC-61583723273115/",
      about: "The Cleaning Queens is a proud, women operated cleaning service! Committed to empowering our communities through professional cleaning. Our focuses are residential maintenance, deep cleans, move in-out & AirBnBs.",
    },
    services: [
      { id: "svc1", name: "Residential Maintenance", price: 109, unit: "per visit", desc: "Weekly, bi-weekly, or monthly upkeep. Same Queen, same royal standard, every time.", icon: "repeat" },
      { id: "svc2", name: "Deep Clean", price: 249, unit: "from", desc: "Baseboards, inside appliances, grout, vents — the top-to-bottom royal reset.", icon: "crown" },
      { id: "svc3", name: "Move In / Move Out", price: 299, unit: "from", desc: "Empty-home clean that gets deposits back and keys handed over with confidence.", icon: "home" },
      { id: "svc4", name: "Airbnb Turnovers", price: 129, unit: "per turn", desc: "Fast, reliable guest-ready turnovers — restocking, staging, and 5-star sparkle.", icon: "building" },
      { id: "svc5", name: "Standard Clean", price: 129, unit: "from", desc: "Every room refreshed: dusting, vacuuming, mopping, kitchen & bath shine.", icon: "sparkle" },
    ],
    reviews: [
      { id: "r0", name: "Esther G.", area: "Facebook", stars: 5, text: "Wow! Amazing!!!", date: d(-5), approved: true },
      { id: "r1", name: "Maria G.", area: "Brookside", stars: 5, text: "My house has never looked this good. The deep clean was worth every penny!", date: d(-4), approved: true },
      { id: "r2", name: "James T.", area: "Overland Park", stars: 5, text: "Bi-weekly service for 8 months now. Always on time, always spotless.", date: d(-9), approved: true },
      { id: "r3", name: "Alicia R.", area: "Waldo", stars: 5, text: "They got our full deposit back on the move-out clean. Landlord was stunned.", date: d(-13), approved: true },
      { id: "r4", name: "Derek W.", area: "River Market", stars: 4, text: "Great office clean, super professional crew. Booking online took 2 minutes.", date: d(-17), approved: true },
      { id: "r5", name: "Sandra K.", area: "Lee's Summit", stars: 5, text: "The team treats my home like their own. Even my picky mother-in-law approved.", date: d(-21), approved: true },
      { id: "r6", name: "Priya N.", area: "Olathe", stars: 5, text: "Post-renovation dust everywhere — gone in one visit. Absolute magicians.", date: d(-26), approved: true },
      { id: "r7", name: "Tom B.", area: "Midtown", stars: 5, text: "Fair pricing, real humans on the phone, and a sparkling kitchen. 10/10.", date: d(-31), approved: true },
      { id: "r8", name: "Chloe M.", area: "Prairie Village", stars: 5, text: "I book everything from my phone and pay online. So easy it feels illegal.", date: d(-36), approved: true },
      { id: "r9", name: "Hank D.", area: "North KC", stars: 4, text: "Solid standard clean. They flagged a leak under my sink I never noticed!", date: d(-40), approved: true },
      { id: "r10", name: "Renee L.", area: "Shawnee", stars: 5, text: "Three years, zero complaints. The Queens run a tight ship.", date: d(-45), approved: true },
      { id: "r11", name: "Marcus P.", area: "Blue Springs", stars: 5, text: "Gift-carded a clean for my mom. She cried. Best money I ever spent.", date: d(-50), approved: false },
    ],
    clients: [
      { id: "c1", name: "Maria Gonzales", phone: "(555) 210-3341", email: "maria.g@example.com", address: "18 Brookside Blvd", plan: "Bi-weekly", ltv: 1740, notes: "Two cats. Prefers eco products. Key under mat (garage)." },
      { id: "c2", name: "James Turner", phone: "(555) 884-1290", email: "jturner@example.com", address: "902 W 87th St, Overland Park", plan: "Bi-weekly", ltv: 1962, notes: "Gate code 4412. Focus on kitchen." },
      { id: "c3", name: "Alicia Reyes", phone: "(555) 337-9084", email: "alicia.r@example.com", address: "77 Waldo Ct", plan: "One-time", ltv: 299, notes: "Move-out done. Send follow-up offer in spring." },
      { id: "c4", name: "Derek Wells / Wells Design Co", phone: "(555) 902-4471", email: "derek@wellsdesign.example.com", address: "310 Delaware St, Suite 200", plan: "Weekly (commercial)", ltv: 4536, notes: "After 6 PM only. Invoice net-15." },
      { id: "c5", name: "Sandra Kim", phone: "(555) 668-2903", email: "skim@example.com", address: "44 Langsford Rd, Lee's Summit", plan: "Monthly", ltv: 981, notes: "Dog (friendly). No bleach on stone counters." },
      { id: "c6", name: "Priya Nair", phone: "(555) 415-7726", email: "priya.n@example.com", address: "1206 Sycamore Dr, Olathe", plan: "One-time", ltv: 349, notes: "Post-construction. Contractor referral — thank them." },
      { id: "c7", name: "Tom Baker", phone: "(555) 573-8810", email: "tbaker@example.com", address: "3117 Main St #4", plan: "Monthly", ltv: 774, notes: "Buzz 204. Loves the lemon finish spray." },
      { id: "c8", name: "Chloe Martin", phone: "(555) 229-6647", email: "chloe.m@example.com", address: "6521 Mission Rd, Prairie Village", plan: "Weekly", ltv: 3268, notes: "Toddler naps 1–3 PM, schedule mornings." },
    ],
    staff: [
      { id: "s1", name: "Queen Latasha", role: "Lead Cleaner", color: "#ff2d92", phone: "(555) 101-2001" },
      { id: "s2", name: "Queen Dana", role: "Cleaner", color: "#c084fc", phone: "(555) 101-2002" },
      { id: "s3", name: "Queen Sofia", role: "Cleaner", color: "#38bdf8", phone: "(555) 101-2003" },
      { id: "s4", name: "Marcus (Ops)", role: "Operations", color: "#facc15", phone: "(555) 101-2004" },
    ],
    jobs: [
      { id: "j1", clientId: "c8", serviceId: "svc1", staffId: "s1", start: d(0, 9), hours: 2, status: "in_progress", recurring: "weekly", checklist: [{ t: "Kitchen", done: true }, { t: "Bathrooms x2", done: true }, { t: "Bedrooms", done: false }, { t: "Floors", done: false }] },
      { id: "j2", clientId: "c1", serviceId: "svc1", staffId: "s2", start: d(0, 13), hours: 3, status: "scheduled", recurring: "biweekly", checklist: [{ t: "Full standard", done: false }, { t: "Eco products only", done: false }] },
      { id: "j3", clientId: "c4", serviceId: "svc4", staffId: "s3", start: d(0, 18), hours: 2.5, status: "scheduled", recurring: "weekly", checklist: [{ t: "Desks + common area", done: false }, { t: "Kitchenette", done: false }, { t: "Trash + recycle", done: false }] },
      { id: "j4", clientId: "c5", serviceId: "svc2", staffId: "s1", start: d(1, 10), hours: 5, status: "scheduled", recurring: null, checklist: [{ t: "Deep clean whole house", done: false }, { t: "No bleach on stone", done: false }] },
      { id: "j5", clientId: "c7", serviceId: "svc5", staffId: "s2", start: d(2, 9), hours: 2, status: "scheduled", recurring: "monthly", checklist: [{ t: "Standard clean", done: false }] },
      { id: "j6", clientId: "c2", serviceId: "svc1", staffId: "s3", start: d(3, 14), hours: 3, status: "scheduled", recurring: "biweekly", checklist: [{ t: "Kitchen focus", done: false }, { t: "Full standard", done: false }] },
      { id: "j7", clientId: "c6", serviceId: "svc2", staffId: "s1", start: d(-2, 8), hours: 6, status: "done", recurring: null, checklist: [{ t: "Debris removal", done: true }, { t: "Dust every surface", done: true }, { t: "Window tracks", done: true }] },
      { id: "j8", clientId: "c3", serviceId: "svc3", staffId: "s2", start: d(-6, 9), hours: 5, status: "done", recurring: null, checklist: [{ t: "Move-out full", done: true }, { t: "Oven + fridge", done: true }] },
      { id: "j9", clientId: "c1", serviceId: "svc1", staffId: "s2", start: d(-14, 13), hours: 3, status: "done", recurring: "biweekly", checklist: [{ t: "Full standard", done: true }] },
      { id: "j10", clientId: "c8", serviceId: "svc1", staffId: "s1", start: d(-7, 9), hours: 2, status: "done", recurring: "weekly", checklist: [{ t: "Full standard", done: true }] },
      { id: "j11", clientId: "c4", serviceId: "svc4", staffId: "s3", start: d(-1, 18), hours: 2.5, status: "done", recurring: "weekly", checklist: [{ t: "Office full", done: true }] },
      { id: "j12", clientId: "c5", serviceId: "svc5", staffId: "s2", start: d(5, 10), hours: 2.5, status: "scheduled", recurring: "monthly", checklist: [{ t: "Standard clean", done: false }] },
      { id: "j13", clientId: "c7", serviceId: "svc2", staffId: "s1", start: d(6, 9), hours: 5, status: "scheduled", recurring: null, checklist: [{ t: "Deep clean", done: false }] },
    ],
    leads: [
      { id: "l1", name: "Beth Howard", phone: "(555) 990-1123", service: "Deep Clean", note: "3bd/2ba in Liberty, wants quote this week", created: d(0, 8) },
      { id: "l2", name: "Carlos Mendez", phone: "(555) 774-0921", service: "Recurring Clean", note: "Weekly, small condo downtown", created: d(-1, 15) },
      { id: "l3", name: "Fran Whitfield", phone: "(555) 336-5540", service: "Move In / Move Out", note: "Closing on the 28th, needs move-in clean", created: d(-2, 11) },
    ],
    invoices: [
      { id: "INV-1041", clientId: "c8", jobId: "j10", amount: 109, status: "paid", issued: d(-7), paidOn: d(-6), method: "card" },
      { id: "INV-1042", clientId: "c4", jobId: "j11", amount: 189, status: "paid", issued: d(-1), paidOn: d(0), method: "ach" },
      { id: "INV-1043", clientId: "c6", jobId: "j7", amount: 349, status: "due", issued: d(-2), paidOn: null, method: null },
      { id: "INV-1044", clientId: "c3", jobId: "j8", amount: 299, status: "paid", issued: d(-6), paidOn: d(-4), method: "card" },
      { id: "INV-1045", clientId: "c1", jobId: "j9", amount: 109, status: "overdue", issued: d(-14), paidOn: null, method: null },
      { id: "INV-1046", clientId: "c8", jobId: "j1", amount: 109, status: "due", issued: d(0), paidOn: null, method: null },
    ],
    bookingRequests: [],
    edits: {},
    theme: { accent: "#ff2d92" },
  };

  /* Demo credentials — fake data, do not ship to production */
  const USERS = [
    { user: "admin", pass: "queen123", name: "Head Queen (Admin)", role: "admin" },
    { user: "staff", pass: "sparkle", name: "Queen Dana", role: "staff" },
  ];

  function load() {
    try {
      const raw = localStorage.getItem(KEY);
      if (raw) {
        const data = JSON.parse(raw);
        if (data.version === VERSION) return data;
      }
    } catch (e) { /* corrupted -> reseed */ }
    const fresh = JSON.parse(JSON.stringify(SEED));
    save(fresh);
    return fresh;
  }

  function save(data) {
    try { localStorage.setItem(KEY, JSON.stringify(data)); } catch (e) { /* quota */ }
  }

  const CQStore = {
    get data() { if (!this._d) this._d = load(); return this._d; },
    commit() { save(this.data); },
    reset() { localStorage.removeItem(KEY); this._d = null; },

    /* ---- auth (demo only) ---- */
    login(user, pass) {
      const u = USERS.find((x) => x.user === user && x.pass === pass);
      if (!u) return null;
      const session = { user: u.user, name: u.name, role: u.role, at: Date.now() };
      localStorage.setItem(SESSION_KEY, JSON.stringify(session));
      return session;
    },
    logout() { localStorage.removeItem(SESSION_KEY); },
    session() {
      try { return JSON.parse(localStorage.getItem(SESSION_KEY) || "null"); }
      catch (e) { return null; }
    },
    isAdmin() { const s = this.session(); return !!s && s.role === "admin"; },

    /* ---- helpers ---- */
    byId(list, id) { return this.data[list].find((x) => x.id === id); },
    money(n) { return new Intl.NumberFormat("en-US", { style: "currency", currency: "USD" }).format(n); },
    dateFmt(iso, opts) {
      return new Intl.DateTimeFormat("en-US", opts || { month: "short", day: "numeric" }).format(new Date(iso));
    },
    timeFmt(iso) {
      return new Intl.DateTimeFormat("en-US", { hour: "numeric", minute: "2-digit" }).format(new Date(iso));
    },
    uid(prefix) { return prefix + Math.random().toString(36).slice(2, 8); },

    /* ---- content edits overlay (admin edit mode) ---- */
    getEdit(key) { return this.data.edits[key]; },
    setEdits(map) { Object.assign(this.data.edits, map); this.commit(); },
    clearEdits() { this.data.edits = {}; this.commit(); },

    /* ---- booking ---- */
    addBooking(req) {
      const item = Object.assign({ id: this.uid("bk"), created: new Date().toISOString(), status: "new" }, req);
      this.data.bookingRequests.push(item);
      this.data.leads.push({ id: this.uid("l"), name: req.name, phone: req.phone, service: req.service, note: req.note || "(website booking)", created: item.created });
      this.commit();
      return item;
    },
  };

  global.CQStore = CQStore;
})(window);
