# Aerospace Specialty Job Shop - Implementation Roadmap

**Project**: Precision CNC/Robotic Manufacturing for Aerospace Market  
**Target**: AS9100 Certified Tier-2/Tier-3 Supplier  
**Timeline**: 24 Months to Full Certification & Revenue Scale  
**Investment**: $40K–$80K Phase 1; $360K–$495K Phase 2  
**Revenue Target**: $300K–$500K Year 1; $1M–$1.5M Year 2  
**Date Created**: 2025-12-11  

---

## 🎯 **Strategic Vision**

**Where We're Going:**
Transform existing CNC/robotic automation capabilities into a certified aerospace supplier serving Tier-1 companies (Boeing, Lockheed, Northrop, Collins Aerospace) with precision-machined components commanding 2–4× premium pricing over commercial work.

**Why Aerospace:**
- **Barrier to Entry**: AS9100 certification + material expertise = moat protecting premium pricing
- **Automation Fit**: Lights-out CNC + robotics maximize ROI on complex, low-volume aerospace parts
- **Market Stability**: Multi-year contracts, recurring orders, recession-resistant defense spending
- **Pricing Power**: $150–$250/hr shop rates vs. $80–$120/hr commercial work

---

## 🧭 **Beyond Machining: Capability & Partner Map**

Aerospace work spans far more than CNC. To win early and scale smart, operate a hybrid model: keep precision machining, metrology, and assemblies in-house; partner out regulated “special processes” to Nadcap-accredited suppliers.

### Core Capability Lanes
- **Precision Machining (In‑House, Now)**: 3–5 axis milling/turning of Al 7075, SS 316/17‑4, Ti‑6Al‑4V; jigs/fixtures; small assemblies.
- **Metrology & QA (In‑House, Near‑Term)**: FAI (AS9102), CMM/optical inspection, gage R&R, ballooned drawings, traceability.
- **Sheet/Tube Fabrication (Partner)**: Sheet metal (brake, hydroform), tube bending, chemfilm conversion coatings.
- **Composites (Partner / Future R&D)**: Prepreg layup, autoclave/oven cure, bonded honeycomb panels.
- **Additive Manufacturing (Partner/Future)**: Metal LPBF/DMLS/EBM for complex geometries; polymer AM for ducts/tooling.
- **Avionics/Electrical (Partner/Future)**: Harness build to IPC/WHMA‑A‑620; EMI/EMC compliant enclosures.

### Nadcap Special Processes (Outsource to Accredited Shops)
Nadcap is the aerospace industry’s accreditation program for critical processes. Common scopes you’ll leverage via partners:
- **Heat Treat** (aluminum, stainless, titanium)
- **Welding/Brazing** (GTAW, resistance, vacuum braze)
- **Chemical Processing** (anodize, alodine/chemfilm, passivation)
- **Coatings** (paint, primers, dry film lube)
- **NDT** (UT, RT/X‑ray, PT, MT, ET)
- **Shot Peen** and surface enhancement

Partner Policy:
- Build an **Approved Special Process Supplier List (ASL)** with current Nadcap certs on file, scope codes, expiration dates.
- Flow down customer/AS9100 requirements on POs (right of access, traceability, certs of conformance, lot/heat tracking).
- Audit/verify at least annually (desk audit acceptable for small shop; on‑site if volume/risk increases).

### Sourcing Plan (Quick View)
- **In‑House (Now)**: CNC machining, basic deburr/finishing, in‑process/final inspection, AS9102 FAI documentation.
- **Near‑Term (0–12 mo)**: CMM or optical metrology, assembly benches, torque/adhesive controls, controlled storage for certs.
- **Partner Network (Now)**: Nadcap heat treat, anodize/passivation, NDT, coatings, sheet/tube fab, AM bureau.
- **Future (12–36 mo)**: 5‑axis cell, composites pilot line, in‑house selective special processes if volume justifies.

### Partner Onboarding Checklist
- Signed NDA + terms; conflict minerals/REACH/ROHS declarations if required
- Provide ballooned drawing + special characteristics; agree inspection plan
- PO includes: spec revs, Nadcap scope, certs required, serialization/traceability, retention period
- Receiving inspection plan: verify dims, cert packages, process stamps, and CoC alignment

---

## 📅 **Phase 1: Foundation & Market Entry (Months 1–12)**

### **Month 1-2: Equipment Audit & Capability Baseline**

#### **Week 1-2: Current State Assessment**
- [ ] **Equipment Inventory**:
  - Document all CNC machines (make, model, capacity, condition)
  - Document robotic systems (robot model, payload, reach, end effectors)
  - Document inspection equipment (calipers, micrometers, height gauge, CMM if any)
  - Photograph shop layout; identify bottlenecks
- [ ] **Capability Matrix**:
  - Materials currently machined (Aluminum 6061, 7075; Steel; Stainless?)
  - Tolerances achievable (±0.001"? ±0.0001"?)
  - Max part size (envelope limitations)
  - Cycle time for reference parts (3 sample parts: simple, medium, complex)
- [ ] **Skills Audit**:
  - CNC programming expertise (CAM software: Fusion 360, Mastercam, SolidCAM?)
  - Robotics integration (ROS2, custom automation?)
  - Inspection/metrology (GD&T knowledge, CMM programming?)
  - Quality management (any ISO 9001 or aerospace experience?)

**Deliverable**: Current State Report (5-10 pages; equipment photos, capability specs, skills matrix)

**Budget**: $0 (internal time: 20–30 hours)

---

#### **Week 3-4: Material Sourcing & Tooling Research**
- [ ] **Aerospace Materials**:
  - **Aluminum 7075-T6**: Source 3 suppliers; get pricing (typically $8–$12/lb)
  - **Titanium Ti-6Al-4V**: Source 2 suppliers; get pricing (typically $30–$60/lb)
  - **Stainless 316/17-4 PH**: Source 2 suppliers; get pricing (typically $5–$15/lb)
  - Verify suppliers can provide **mill certs** (material certification required for AS9100)
- [ ] **Tooling for Hard Materials**:
  - Carbide end mills for Titanium (low-helix, high-rigidity)
  - High-pressure coolant system (for Titanium: 1,000+ PSI through-spindle coolant)
  - Specialty tooling (thread mills, chamfer mills, ball-nose for 3D surfaces)
- [ ] **Supplier Relationships**:
  - Contact 5–10 aerospace material suppliers (McMaster-Carr, Online Metals, Aircraft Spruce, specialty distributors)
  - Request sample quotes (small orders: 1–5 lbs each material)

**Deliverable**: Approved Supplier List (ASL) draft with 3–5 material suppliers

**Budget**: $500–$1,000 (sample material purchases for testing)

---

### **Month 3-4: Portfolio Development & First Sample Parts**

#### **Week 5-8: Machine 10 Portfolio Parts**
**Objective**: Build showcase parts demonstrating aerospace-relevant capabilities

**Sample Part List** (choose 10 from these categories):
1. **Structural Brackets** (Aluminum 7075-T6):
   - L-bracket with gusset (2.5D milling, multiple setups)
   - Lightweighting pockets (3D contour milling)
2. **Fittings & Adapters**:
   - Threaded adapter (turning + milling; demonstrate GD&T: concentricity, perpendicularity)
   - AN-style hydraulic fitting (tight tolerances: ±0.001")
3. **Complex Geometry** (demonstrate 3-axis or 5-axis if available):
   - Turbine blade mockup (thin walls, complex curves)
   - Impeller hub (radial features, contoured surfaces)
4. **Titanium Sample**:
   - Simple bracket or bushing (prove ability to machine Ti-6Al-4V)
5. **Stainless Steel**:
   - Shaft or flange (demonstrate turning + milling; surface finish <32 Ra)

**Process for Each Part:**
- [ ] CAD model (SolidWorks, Fusion 360, or customer-provided STEP file)
- [ ] CAM programming (generate toolpaths, simulate to avoid collisions)
- [ ] First article machining (slow feeds/speeds; measure every dimension)
- [ ] Inspection report (record all dimensions; note any deviations)
- [ ] Photography (high-quality photos from 3 angles; show surface finish, details)

**Deliverable**: Portfolio binder (PDF or physical) with 10 parts; each part includes:
- CAD drawing with GD&T callouts
- Inspection report (all dimensions measured)
- Photos (3 angles + close-ups of critical features)

**Budget**: $1,000–$2,000 (material, tooling wear, photography setup)

**Timeline**: 2–4 hours per part (40–80 hours total; spread over 4 weeks)

---

### **Month 5-6: Quality Foundation (ISO 9001 Practices)**

#### **Week 9-12: Build Basic QMS Infrastructure**
**Objective**: Establish ISO 9001 practices (AS9100 prerequisite)

**Documents to Create:**
- [ ] **Quality Policy** (1 page):
  - "ABC Machining is committed to delivering precision aerospace components that meet customer specifications and AS9100 standards through continuous improvement."
- [ ] **Organizational Chart** (1 page):
  - Define roles: Owner, Quality Manager (can be same person), CNC Programmer, Operator, Inspector
- [ ] **Process Map** (1 page):
  - Flowchart: Quote → Contract Review → Production → Inspection → Shipping
- [ ] **Document Control Procedure** (2-3 pages):
  - How to create, approve, revise, archive documents (use simple numbering: QP-001, WI-001, etc.)
- [ ] **Calibration Procedure** (2-3 pages):
  - Calibration intervals (annual for most equipment)
  - Traceable standards (NIST or A2LA accredited lab)
  - Out-of-tolerance actions (quarantine parts measured with bad equipment)
- [ ] **Job Traveler Template** (1-page form):
  - Part number, customer, material, operations checklist, inspection checkpoints, sign-offs

**Physical Actions:**
- [ ] **Calibration**: Send all measurement equipment to cal lab (cost: $50–$150 per tool)
  - Calipers, micrometers, height gauge, indicators, etc.
  - Affix calibration stickers (due date visible)
- [ ] **Calibration Log**: Spreadsheet or database tracking cal due dates
- [ ] **Training Matrix**: List employees and required skills (CNC programming, inspection, GD&T, AS9100)
- [ ] **Training Records**: Document dates of any training completed (online courses, vendor training, etc.)

**Software Tools (Optional but Helpful):**
- [ ] **Document Management**: Google Drive with version control OR QMS software (ETQ, MasterControl; $100–$500/month)
- [ ] **Calibration Tracking**: Simple Excel/Google Sheets or dedicated tool (CalFusion, GAGEtrak)

**Deliverable**: QMS Folder (10–15 documents + calibration records + training matrix)

**Budget**: $1,000–$2,000 (calibration lab fees) + $0–$500 (software if using paid tools)

**Timeline**: 30–40 hours (document writing, calibration coordination)

---

### **Month 7-8: Customer Outreach & First Quotes**

#### **Week 13-16: Identify & Contact Aerospace Prospects**
**Objective**: Land 3–5 initial customers (prototype or small batch work)

**Target Customer List** (build list of 50 prospects):
1. **Aerospace Tier-2 Suppliers** (within 300-mile radius):
   - Search LinkedIn: "aerospace manufacturing," "precision machining," "AS9100"
   - Dun & Bradstreet, ThomasNet directories
   - Industry associations: NDIA, SAE, local chambers of commerce
2. **Engineering Firms** (aerospace consulting, R&D labs):
   - Prototyping work (no AS9100 required initially; builds relationship)
3. **University Aerospace Programs**:
   - Research labs, student rocket teams (AIAA competitions), UAV projects
4. **Defense Contractors** (if ITAR feasible):
   - Small defense primes or subs (avoid large OEMs initially; too much bureaucracy)

**Outreach Campaign:**
- [ ] **Email Template** (personalized for each prospect):
  - Subject: "Precision CNC Machining for Aerospace Prototypes & Production"
  - Body: Introduce capabilities (3-axis CNC, robotic automation, materials: Al 7075, Ti, SS)
  - Attach: Portfolio PDF (10 sample parts from Month 3-4)
  - Call-to-action: "Can I quote on your next project?"
- [ ] **LinkedIn Engagement**:
  - Connect with procurement managers, quality managers, engineering managers
  - Share technical content (post photos of sample parts, machining videos, material comparisons)
- [ ] **Phone Follow-Up**:
  - Call 48 hours after email; leave voicemail if no answer
  - Script: "I'm reaching out because we specialize in low-volume aerospace machining. I sent a portfolio—would you have 10 minutes to discuss potential fit?"

**Cold Outreach Funnel:**
- 50 emails → 10 responses (20% open rate) → 3 meetings → 1–2 RFQs (requests for quote)

**Quote Strategy:**
- [ ] First quotes: **At-cost or slight loss** (goal is portfolio building + relationship)
- [ ] Typical: Material cost × 1.3 + machine time × $100–$130/hr (below target $150–$250/hr)
- [ ] Highlight: "Fast turnaround (2 weeks), local production, ready for AS9100 certification by Month 18"

**Deliverable**: 
- Prospect tracking spreadsheet (50 contacts, outreach dates, responses, RFQs)
- 3–5 quotes submitted (even if not all win)

**Budget**: $500–$1,000 (portfolio printing/mailing, LinkedIn premium, CRM tool if needed)

**Timeline**: 4 weeks (3–5 hours/week prospecting + follow-up)

---

### **Month 9-10: First Customers & Process Refinement**

#### **Week 17-20: Execute First Paying Jobs**
**Objective**: Deliver 3–5 jobs flawlessly; build case studies

**Production Workflow (per job):**
- [ ] **Contract Review**:
  - Customer PO (purchase order) received
  - Technical review: Can we meet tolerances? Materials available? Delivery date feasible?
  - Signed contract review checklist (document approval to proceed)
- [ ] **Material Procurement**:
  - Order material with mill cert (certification of material properties, heat lot)
  - Receive, inspect (verify dimensions, no damage), log in inventory
- [ ] **CAM Programming**:
  - Generate toolpaths; simulate (Vericut, Mastercam Simulator, Fusion 360 Simulation)
  - First article programming (conservative feeds/speeds; leave extra stock for manual finishing if needed)
- [ ] **Production**:
  - Machine first part; full inspection (measure every dimension)
  - If good: run remaining quantity (lights-out if possible)
  - If issues: corrective action (adjust program, tooling, fixturing)
- [ ] **Inspection**:
  - Final inspection per job traveler checklist
  - Inspection report (record all critical dimensions; note any deviations)
  - Customer approval if first article (send photos + inspection report)
- [ ] **Packaging & Shipping**:
  - Anti-corrosion packaging (VCI bags for metals)
  - Label: part number, quantity, customer name, PO number
  - Ship with packing slip + inspection report + mill cert

**Post-Delivery:**
- [ ] **Customer Feedback**:
  - Email survey: "How did we do? Any issues? Would you order again?"
  - Request testimonial (for website, LinkedIn)
- [ ] **Case Study**:
  - Document: Customer, part type, material, challenges, solution, results
  - Photos + metrics (e.g., "Delivered 50 aerospace brackets in 10 days, 100% on-spec, 0 rejects")

**Deliverable**: 
- 3–5 completed jobs
- 3 customer testimonials
- 3 case studies (1-page each)

**Budget**: Material + tooling costs passed through to customer; net profit: $500–$2,000 per job (low margin initially)

**Revenue Target**: $5,000–$15,000 total (Month 9-10 combined)

---

### **Month 11-12: AS9100 Preparation Kickoff**

#### **Week 21-24: Hire Consultant & Gap Analysis**
**Objective**: Begin AS9100 certification process (12–18 month timeline)

**Consultant Selection:**
- [ ] **RFP to 3–5 Consultants**:
  - Search: "AS9100 consultant [your state]"
  - Requirements: Aerospace experience, small shop experience (not just large OEMs), references
  - Budget: $10,000–$20,000 for 6–12 months (includes gap analysis, QMS development, mock audits)
- [ ] **Interview & Select**:
  - Ask: "How many small shops (<10 employees) have you certified?"
  - Ask: "What's your approach? Do you write procedures or teach us to write them?" (Want: teach)
  - Check references (call 2–3 past clients)

**Gap Analysis:**
- [ ] **Consultant Visit** (1 day on-site):
  - Review current QMS (documents from Month 5-6)
  - Observe operations (machining, inspection, material handling)
  - Interview key personnel (owner, quality manager, operators)
- [ ] **Gap Analysis Report**:
  - Compare current state vs. AS9100 requirements (10 clauses)
  - Identify missing procedures (e.g., configuration management, CAPA, supplier management)
  - Prioritize gaps (critical vs. nice-to-have)
- [ ] **Implementation Plan**:
  - Timeline: 12–15 months to certification audit
  - Milestones: QMS development (Months 13-18), internal audits (Month 19-21), certification audit (Month 22-24)
  - Assignments: Who writes which procedures? (Consultant drafts; you review/revise)

**Deliverable**: 
- Consultant contract signed
- Gap analysis report (15–25 pages)
- AS9100 implementation plan (Gantt chart or timeline)

**Budget**: $2,000–$5,000 (consultant retainer deposit)

**Timeline**: 4 weeks (RFP, selection, gap analysis)

---

### **Phase 1 Summary: Month 12 Checkpoint**

**Achievements:**
✅ Equipment & capabilities documented  
✅ 10-part portfolio built (Aluminum, Titanium, Stainless)  
✅ Basic QMS in place (ISO 9001 practices)  
✅ 3–5 paying customers acquired  
✅ $10K–$30K revenue generated  
✅ AS9100 consultant hired; gap analysis complete  

**Metrics:**
| Metric | Target | Actual |
|--------|--------|--------|
| **Sample Parts Built** | 10 | ___ |
| **Customers Acquired** | 3–5 | ___ |
| **Revenue (Month 1-12)** | $10K–$30K | $___ |
| **Jobs Completed** | 5–10 | ___ |
| **On-Time Delivery** | 90%+ | ___% |
| **First Pass Yield** | 85%+ | ___% |

**Financials:**
- **Investment**: $5,000–$10,000 (materials, calibration, consultant deposit)
- **Revenue**: $10,000–$30,000
- **Net**: $0–$20,000 (break-even to small profit; primary goal is portfolio + customer acquisition)

**Readiness for Phase 2**: ✅ Ready to scale if customer feedback positive and 3+ repeat customers secured

---

## 📅 **Phase 2: AS9100 Certification & Scale (Months 13–24)**

### **Month 13-15: QMS Development (Procedures & Work Instructions)**

#### **Week 25-36: Write AS9100 Documentation**
**Objective**: Develop complete Quality Management System

**Core Documents** (20–30 procedures typical):

**Management Procedures:**
- [ ] QP-001: Document Control (create, approve, revise, archive)
- [ ] QP-002: Record Control (identify, store, protect, dispose)
- [ ] QP-003: Management Review (agenda, frequency, inputs, outputs)
- [ ] QP-004: Internal Audit (planning, execution, reporting)
- [ ] QP-005: Corrective Action (CAPA process, root cause analysis)
- [ ] QP-006: Risk Management (PFMEA for key processes)

**Production Procedures:**
- [ ] QP-010: Production Planning & Control (scheduling, work instructions)
- [ ] QP-011: Configuration Management (engineering changes, revision control)
- [ ] QP-012: Material Control (receiving, storage, traceability, FIFO)
- [ ] QP-013: Production Equipment Maintenance (preventive maintenance schedule)
- [ ] QP-014: Tool Control (tool crib, tool life tracking, replacement)

**Quality Procedures:**
- [ ] QP-020: Calibration (equipment, intervals, traceable standards)
- [ ] QP-021: Inspection & Testing (receiving, in-process, final)
- [ ] QP-022: First Article Inspection (AS9102 process)
- [ ] QP-023: Nonconforming Product (quarantine, disposition, NCR process)
- [ ] QP-024: Customer Property (control of customer-furnished material/tooling)

**Supply Chain Procedures:**
- [ ] QP-030: Supplier Management (approved supplier list, evaluation, audits)
- [ ] QP-031: Purchasing (PO requirements: specs, certs, right of access)
- [ ] QP-032: Receiving Inspection (verify material, dimensions, certs)

**Customer Procedures:**
- [ ] QP-040: Contract Review (review requirements before accepting order)
- [ ] QP-041: Customer Communication (order entry, changes, complaints)
- [ ] QP-042: Product Identification & Traceability (lot tracking, serialization)

**Work Instructions** (as needed for complex tasks):
- [ ] WI-001: CNC Setup Procedure (vise alignment, tool offsets, work coordinates)
- [ ] WI-002: CMM Inspection Procedure (if CMM available)
- [ ] WI-003: Titanium Machining Best Practices (feeds, speeds, coolant)
- [ ] WI-004: AS9102 FAI Completion (how to fill out Form 1, 2, 3)

**Forms/Templates:**
- [ ] Job Traveler (route sheet with inspection checkpoints)
- [ ] Nonconformance Report (NCR)
- [ ] Corrective Action Request (CAR)
- [ ] AS9102 Forms (Form 1: Part Accountability; Form 2: Product Accountability; Form 3: Characteristic Accountability)
- [ ] Calibration Log
- [ ] Training Matrix
- [ ] Internal Audit Checklist
- [ ] Supplier Evaluation Form

**Process:**
- [ ] Consultant drafts procedures (based on gap analysis)
- [ ] You review & customize (make them fit your actual operations; not generic)
- [ ] Implement trial run (use procedures for 1–2 jobs; identify issues)
- [ ] Revise based on feedback

**Deliverable**: QMS Folder with 20–30 procedures + 10–15 forms (100–150 pages total)

**Budget**: $5,000–$10,000 (consultant fees for this phase)

**Timeline**: 3 months (concurrent with production; consultant works remotely; you implement)

---

### **Month 16-18: Implementation & Training**

#### **Week 37-48: Put QMS into Daily Practice**
**Objective**: Operate per AS9100 procedures for 3–6 months (auditors want history)

**Training:**
- [ ] **All-Hands QMS Training** (4-hour session):
  - Quality policy & objectives
  - Everyone's role in the QMS
  - How to use job travelers, NCRs, CARs
  - Consequences of nonconformance (aircraft safety)
- [ ] **Specialized Training**:
  - CNC Programmer: Configuration management, PFMEA
  - Inspector: GD&T, AS9102 FAI process, calibration
  - Quality Manager: Internal auditing, CAPA, supplier management

**Daily Operations:**
- [ ] **Job Travelers**: Every job has traveler; all inspections documented
- [ ] **Traceability**: Material certs filed; lot tracking in place (can you trace finished part to raw material heat lot?)
- [ ] **Calibration**: All measurement equipment current; calibration log updated
- [ ] **Nonconformances**: Any defects documented on NCR; root cause analysis; corrective action
- [ ] **Management Review**: Monthly meeting (review KPIs, NCRs, customer feedback, audit results)

**Software Implementation (if not already done):**
- [ ] **ERP/MES**: Production tracking, traceability (IQMS, Plex, E2 Shop System; $500–$2,000/month)
- [ ] **QMS Software**: Document control, CAPA, audits (ETQ, MasterControl; $200–$1,000/month)
- [ ] OR: Continue with spreadsheets/Google Docs if volume low (<20 jobs/month)

**Deliverable**: 
- 3–6 months of records (30–60 job travelers, 5–10 NCRs, 2–3 management review minutes)
- All employees trained (training records signed)

**Budget**: $2,000–$5,000 (training courses, software subscriptions)

**Timeline**: 3 months (concurrent with production)

---

### **Month 19-21: Internal Audits & Pre-Assessment**

#### **Week 49-60: Self-Audit QMS**
**Objective**: Identify issues before certification audit

**Internal Auditor Training:**
- [ ] Train 2–3 employees as internal auditors
  - Course: AS9100 Internal Auditor (2-day course; $1,000–$1,500 per person)
  - Online options: ASQ, Exemplar Global, consultant-led

**Internal Audit Schedule:**
- [ ] **Audit 1** (Month 19): Audit 50% of QMS (production, inspection, calibration)
- [ ] **Audit 2** (Month 20): Audit remaining 50% (management, supplier management, CAPA)
- [ ] **Audit 3** (Month 21): Full QMS audit (all processes; both shifts if applicable)

**Audit Process:**
- [ ] **Prepare Checklist**: Use AS9100 clause-by-clause checklist (available from consultant or registrar)
- [ ] **Conduct Audit**:
  - Interview employees (Do they know procedures? Follow them?)
  - Observe operations (Is traceability working? Calibration current?)
  - Review records (Job travelers complete? NCRs have root cause?)
- [ ] **Document Findings**:
  - Major Nonconformances: Critical gaps (e.g., no calibration, no traceability)
  - Minor Nonconformances: Small issues (e.g., missing signature on one form)
  - Observations: Opportunities for improvement
- [ ] **Corrective Actions**:
  - Fix all Major NCs immediately
  - Fix Minor NCs within 30 days
  - Verify effectiveness (re-audit corrected areas)

**Pre-Assessment Audit** (Optional; Month 21):
- [ ] Registrar conducts 1-day mock audit
- [ ] Informal feedback (not pass/fail)
- [ ] Address gaps before formal certification audit

**Deliverable**: 
- 3 internal audit reports
- All NCs corrected (evidence documented)
- Optional: Pre-assessment report from registrar

**Budget**: $3,000–$5,000 (auditor training + pre-assessment fee if pursued)

**Timeline**: 3 months (1 audit per month)

---

### **Month 22-24: AS9100 Certification Audit**

#### **Week 61-72: Registrar Certification Audit**
**Objective**: Achieve AS9100 Rev D certification

**Registrar Selection** (should start in Month 18-19):
- [ ] **Get Quotes from 3 Registrars**:
  - SAI Global, NQA, Perry Johnson Registrars, BSI Group
  - Compare: cost, timeline, auditor experience, surveillance audit frequency
- [ ] **Select Registrar**:
  - Criteria: Cost ($10K–$25K), reputation, auditor assigned to your audit (aerospace experience?)
- [ ] **Submit Application**:
  - Scope: "CNC machining of aerospace brackets, fittings, and structural components"
  - Provide: Quality Manual, procedures list, company info

**Stage 1 Audit** (Month 22; 1 day, remote or on-site):
- **Focus**: Document review (Quality Manual, procedures, records)
- **Auditor Checks**: 
  - Do procedures cover all AS9100 requirements?
  - Are records available (job travelers, NCRs, calibration, training)?
  - Any obvious gaps?
- **Outcome**: "Ready for Stage 2" OR "Address deficiencies" (fix and re-submit)

**Stage 2 Audit** (Month 23; 2–3 days on-site):
- **Focus**: Implementation audit (Is QMS actually working?)
- **Day 1**: Opening meeting; facility tour; document review
- **Day 2**: Interviews with employees; observe operations; review records
  - **Questions Auditor Asks**:
    - Employee: "What's your role? How do you contribute to quality?"
    - Operator: "Where's the procedure for this operation? Show me the job traveler."
    - Inspector: "How do you verify this dimension? When was this caliper calibrated?"
    - Quality Manager: "Show me a recent NCR. What was root cause? Corrective action? Verified effective?"
  - **Auditor Checks**:
    - Traceability: Pick a finished part; trace back to raw material cert (heat lot, mill cert)
    - Calibration: Randomly check 5 tools; all stickers current?
    - Job travelers: Complete? Signed at each step?
    - NCRs: Root cause analysis documented? Corrective action implemented?
- **Day 3**: Final observations; closing meeting; findings presentation

**Findings:**
- **Major Nonconformances**: Must fix before certification issued (30-day timeline typical)
  - Example: "No calibration records found for 3 micrometers"
- **Minor Nonconformances**: Fix within 90 days (can certify with open Minors)
  - Example: "One job traveler missing inspector signature"
- **Observations**: Nice-to-haves; no action required
  - Example: "Consider adding SPC charts for key characteristics"

**Post-Audit Corrective Actions** (Month 24):
- [ ] Fix all Major NCs immediately
- [ ] Provide evidence to registrar (photos, revised procedures, new records)
- [ ] Registrar reviews evidence remotely
- [ ] If satisfied: **Certification issued**

**Certification Issued** (Month 24 target):
- [ ] **AS9100 Rev D Certificate**: Valid 3 years
- [ ] **Surveillance Audits**: Annual 1-day audits (Years 1, 2); recertification audit (Year 3)
- [ ] **Marketing**: Add "AS9100 Certified" to website, LinkedIn, business cards, quotes

**Deliverable**: AS9100 Rev D Certification

**Budget**: $10,000–$25,000 (Stage 1 + Stage 2 + certificate issuance)

**Timeline**: 3 months (1 month prep, 1 month audit, 1 month corrective actions)

---

### **Month 22-24: Aerospace Market Entry (Post-Certification)**

#### **Concurrent with Certification Audit: Ramp Up Marketing**

**Immediate Actions (Day 1 After Certification):**
- [ ] **Update All Marketing**:
  - Website homepage: "AS9100 Rev D Certified Aerospace Supplier"
  - LinkedIn company page: Update description, post announcement
  - Email signature: Add "AS9100 Certified" badge
- [ ] **Press Release**:
  - Local business journal, industry publications (Manufacturing News, SAE Magazines)
  - "ABC Machining Achieves AS9100 Certification; Now Serving Aerospace OEMs"
- [ ] **Customer Notifications**:
  - Email all existing customers: "We're now AS9100 certified—ready for your aerospace projects"
  - Call top 3–5 customers personally

**Targeted Aerospace Outreach:**
- [ ] **Join Industry Associations**:
  - NDIA (National Defense Industrial Association): $500–$1,000/year membership
  - SAE International (Aerospace): $150–$300/year
  - Attend local chapter meetings (networking)
- [ ] **OASIS Registration**:
  - OASIS (Online Aerospace Supplier Information System): Free listing
  - Aerospace OEMs search OASIS for certified suppliers
  - Complete profile: capabilities, certifications, equipment, materials
- [ ] **Tier-1 Supplier Qualification**:
  - Target 5–10 Tier-1 aerospace suppliers (local or regional)
  - Submit capability statement (includes: AS9100 cert, equipment list, sample parts, quality manual excerpt)
  - Request supplier qualification audit (expect 1-day audit by customer quality team)

**Trade Show Presence:**
- [ ] **Exhibit or Attend**:
  - Paris Air Show (international; every 2 years; expensive; $10K–$50K booth)
  - Farnborough Air Show (UK; alternate years)
  - SAE AeroTech Congress (US; annual; $2K–$10K booth)
  - Regional aerospace expos (cheaper; $500–$2K booth)
- [ ] **Booth Strategy**:
  - Display: 10 sample parts (mounted, labeled with material + tolerance)
  - Handout: Capability brochure (1-page: equipment, materials, certifications, contact)
  - Lead capture: Business cards, QR code to quote request form

**Cold Outreach Campaign (Aerospace-Focused):**
- [ ] **Build List**: 100 aerospace prospects (Tier-1 suppliers, OEMs, defense contractors)
- [ ] **Email Campaign**:
  - Subject: "AS9100 Certified Supplier for Precision Aerospace Machining"
  - Body: Introduce capabilities; highlight certification; offer to quote
  - Attach: Capability statement + portfolio PDF
- [ ] **LinkedIn Outreach**:
  - Connect with 50 procurement managers, quality managers at aerospace companies
  - Personalized message: "Saw you're hiring suppliers for [part type]—we're now AS9100 certified and would love to quote"
- [ ] **Phone Follow-Up**:
  - Call within 48 hours of email; reference certification

**Expected Results:**
- 100 outreach → 20 responses → 5 supplier qualification requests → 2–3 approved vendors → 1–2 POs

**Deliverable**:
- 5–10 aerospace customers in pipeline
- 2–3 aerospace jobs quoted or in production

**Budget**: $5,000–$15,000 (trade show booth, memberships, travel, marketing materials)

**Timeline**: Concurrent with Month 22-24 (start as soon as certification confirmed)

---

### **Phase 2 Summary: Month 24 Checkpoint**

**Achievements:**
✅ AS9100 Rev D certified  
✅ Full QMS operational (20–30 procedures, 6+ months of records)  
✅ 10–15 total customers (5+ aerospace)  
✅ $80K–$150K/month revenue (aerospace jobs at $150–$250/hr)  
✅ Approved supplier for 2–3 Tier-1 aerospace companies  

**Metrics:**
| Metric | Target | Actual |
|--------|--------|--------|
| **AS9100 Certified** | ✅ Yes | ___ |
| **Aerospace Customers** | 5+ | ___ |
| **Monthly Revenue** | $80K–$150K | $___ |
| **Shop Rate (Aerospace Jobs)** | $150–$250/hr | $___ |
| **Machine Utilization** | 60–75% | ___% |
| **On-Time Delivery** | 95%+ | ___% |
| **First Pass Yield** | 95%+ | ___% |

**Financials:**
- **Investment (Phase 2)**: $30,000–$60,000 (consultant, audits, training, marketing)
- **Revenue (Year 2)**: $1,000,000–$1,500,000
- **Gross Margin**: 55–70% (aerospace premium pricing)
- **Operating Profit**: 20–35% ($200K–$525K)
- **ROI**: Phase 2 investment pays back in 2–4 months

**Readiness for Phase 3**: ✅ Ready to scale (add capacity, staff, diversify into medical/defense)

---

## 📅 **Phase 3: Scale & Diversify (Months 25–36)**

### **Month 25-28: Capacity Expansion**

#### **Objective**: Add second CNC cell to increase throughput

**Equipment Purchase:**
- [ ] **Second CNC Machine**: 3-axis or 5-axis mill ($150K–$300K)
  - 5-axis recommended if aerospace demand includes complex geometries (turbine blades, impellers)
  - Brands: Haas, Mazak, DMG Mori, Okuma
- [ ] **Robotic Material Handling**: UR10 or ABB IRB 1200 ($30K–$60K)
- [ ] **Tooling Package**: 60-tool magazine, tool holders, cutting tools ($20K–$40K)
- [ ] **Coolant System**: High-pressure through-spindle (for Titanium; $10K–$20K)
- [ ] **Pallet System** (optional): Automated pallet changer for lights-out queueing ($30K–$80K)

**Financing:**
- [ ] **Equipment Loan**: 5–7 year term at 5–7% interest
  - Monthly payment: $3,500–$6,000 (for $300K financed at 6% over 7 years)
  - Payback: If new machine generates $50K–$80K/month revenue (60–75% utilization), loan self-funds

**Facility Upgrades:**
- [ ] **Floor Space**: Ensure adequate space (second machine + robot = 10' × 12' footprint)
- [ ] **Electrical**: 480V 3-phase service (verify capacity; may need panel upgrade)
- [ ] **Compressed Air**: Additional capacity if robotic gripper pneumatic
- [ ] **Coolant Disposal**: Titanium machining generates more waste; verify disposal capacity

**Timeline:** 3–4 months (quote, order, delivery, installation, runoff)

**Deliverable**: Second CNC cell operational; capacity doubled

**Budget**: $300K–$400K (machine, robot, tooling, installation)

---

### **Month 29-32: Team Expansion**

#### **Objective**: Hire key staff to support scale

**Hires:**
- [ ] **Quality Engineer** (Salary: $60K–$80K/year):
  - Responsibilities: AS9100 compliance, internal audits, CAPA, supplier audits
  - Requirements: AS9100 experience, aerospace background, CQE (Certified Quality Engineer) preferred
- [ ] **CNC Programmer** (Salary: $55K–$75K/year):
  - Responsibilities: CAM programming, toolpath optimization, first article programming
  - Requirements: Mastercam/Fusion/SolidCAM expert, 5+ years CNC experience, aerospace preferred
- [ ] **Production Technician** (Salary: $40K–$55K/year):
  - Responsibilities: Machine operation, setup, in-process inspection, deburring/finishing
  - Requirements: CNC operation experience, blueprint reading, micrometer/caliper proficiency

**Hiring Process:**
- [ ] Job postings: LinkedIn, Indeed, local technical colleges, trade schools
- [ ] Interview: Technical assessment (give them a drawing; ask how they'd machine it)
- [ ] Reference checks (critical for aerospace quality culture fit)

**Training:**
- [ ] Onboard to QMS: 2-day training (quality policy, procedures, AS9100 expectations)
- [ ] Shadowing: 1–2 weeks with existing staff
- [ ] Ongoing: Quarterly training (GD&T, new equipment, process improvements)

**Timeline**: 4 months (recruiting, hiring, training)

**Deliverable**: 3 new hires fully onboarded

**Budget**: $155K–$210K/year (salaries) + $10K–$20K (recruiting, training, benefits ramp-up)

---

### **Month 33-36: Diversification & Strategic Accounts**

#### **Objective**: Expand into adjacent markets; secure multi-year contracts

**Market Diversification:**
- [ ] **Medical Devices** (if ISO 13485 feasible):
  - Target: Surgical instruments, implant components
  - Requirements: Consider ISO 13485 certification (6–12 month process; $20K–$40K)
  - Revenue Potential: +$30K–$60K/month
- [ ] **Defense** (if ITAR feasible):
  - Target: Weapon system components, vehicle parts
  - Requirements: ITAR registration ($2,250/year), facility security
  - Revenue Potential: +$50K–$100K/month (defense contracts often larger)
- [ ] **Commercial Aerospace** (continue growing):
  - Target: Business jets, UAVs, space launch
  - Requirements: Already AS9100 certified; expand customer base

**Strategic Account Development:**
- [ ] **Target 3–5 Tier-1 Suppliers**:
  - Goal: Become preferred supplier (70%+ of their outsourced machining)
  - Strategy: Prove capability with 5–10 small jobs; propose annual supply agreement
- [ ] **Long-Term Agreements**:
  - 2–3 year supply agreements with price escalation clauses (protect margin)
  - Blanket POs (customer commits to annual volume; you guarantee capacity)
  - Consignment inventory (stock parts for customer JIT needs; paid on pull)

**5-Axis Capability** (if added in Month 25-28):
- [ ] **Market Entry**: Complex aerospace geometries (turbine blades, impellers, manifolds)
- [ ] **Pricing**: $200–$300/hr (premium over 3-axis due to complexity)
- [ ] **Customers**: Jet engine OEMs, aerospace Tier-1s with high-value parts

**Deliverable**:
- 2–3 long-term supply agreements signed
- Revenue split: 50% aerospace, 25% medical, 25% defense (or adjust based on feasibility)

**Budget**: $20K–$40K (ISO 13485 if pursued, ITAR registration, sales/BD travel)

**Timeline**: 4 months

---

### **Phase 3 Summary: Month 36 Checkpoint**

**Achievements:**
✅ Second CNC cell operational (capacity doubled)  
✅ 3 new hires (Quality Engineer, Programmer, Technician)  
✅ 20–30 active customers across aerospace, medical, defense  
✅ 2–3 multi-year supply agreements  
✅ $150K–$250K/month revenue  

**Metrics:**
| Metric | Target | Actual |
|--------|--------|--------|
| **CNC Cells Operational** | 2 | ___ |
| **Employees** | 6–8 | ___ |
| **Monthly Revenue** | $150K–$250K | $___ |
| **Active Customers** | 20–30 | ___ |
| **Machine Utilization** | 70–85% | ___% |
| **Customer Retention** | 80%+ | ___% |

**Financials (Year 3 Annual):**
- **Revenue**: $1.8M–$3M
- **Gross Margin**: 60–70%
- **Operating Profit**: 25–35% ($450K–$1M)
- **Owner Compensation**: $150K–$300K (salary + distributions)

---

## 💰 **Financial Summary (24-Month Investment & Returns)**

### **Investment Breakdown**

| Phase | Timeline | Investment | Purpose |
|-------|----------|------------|---------|
| **Phase 1** | Months 1-12 | $40K–$80K | Portfolio, QMS, consultant, marketing |
| **Phase 2** | Months 13-24 | $30K–$60K | AS9100 certification, audits, training |
| **Phase 3** | Months 25-36 | $360K–$495K | Second CNC cell, team expansion, diversification |
| **Total (3 years)** | Months 1-36 | **$430K–$635K** | Full build-out to $1.8M–$3M/year revenue |

### **Revenue Trajectory**

| Period | Monthly Revenue | Annual Revenue | Cumulative Revenue |
|--------|----------------|----------------|-------------------|
| **Months 1-6** | $5K–$10K | N/A | $30K–$60K |
| **Months 7-12** | $15K–$40K | $120K–$300K | $150K–$360K |
| **Year 2** (Months 13-24) | $60K–$150K | $900K–$1.5M | $1.05M–$1.86M |
| **Year 3** (Months 25-36) | $150K–$250K | $1.8M–$3M | $2.85M–$4.86M |

### **Profitability**

| Year | Revenue | Gross Margin | Operating Profit | ROI |
|------|---------|--------------|------------------|-----|
| **Year 1** | $150K–$360K | 45–55% | ($10K)–$50K | (15%)–125% |
| **Year 2** | $900K–$1.5M | 55–70% | $200K–$525K | 300–750% |
| **Year 3** | $1.8M–$3M | 60–70% | $450K–$1M | 700–1,500% |

**ROI Calculation** (Year 2):
- Investment (Phases 1+2): $70K–$140K
- Operating Profit (Year 2): $200K–$525K
- ROI: 140–375% in Year 2 alone

**Payback Period**: 12–18 months (certification investment recovers via aerospace premium pricing)

---

## 📊 **Key Performance Indicators (KPIs) - Tracking Dashboard**

### **Financial KPIs**
| Metric | Month 12 | Month 24 | Month 36 |
|--------|----------|----------|----------|
| **Monthly Revenue** | $25K–$40K | $100K–$150K | $200K–$250K |
| **Gross Margin** | 50–60% | 60–70% | 65–70% |
| **Operating Profit Margin** | 5–15% | 20–35% | 25–35% |
| **Shop Rate (Average)** | $110–$150/hr | $160–$220/hr | $180–$250/hr |

### **Operational KPIs**
| Metric | Month 12 | Month 24 | Month 36 |
|--------|----------|----------|----------|
| **Machine Utilization** | 40–60% | 65–75% | 75–85% |
| **First Pass Yield** | 90%+ | 95%+ | 97%+ |
| **On-Time Delivery** | 90%+ | 95%+ | 97%+ |
| **Scrap Rate** | <5% | <3% | <2% |
| **Lights-Out Hours (% of total)** | 20–30% | 40–50% | 50–60% |

### **Customer KPIs**
| Metric | Month 12 | Month 24 | Month 36 |
|--------|----------|----------|----------|
| **Active Customers** | 5–10 | 15–25 | 25–40 |
| **Aerospace Customers** | 2–3 | 8–12 | 15–25 |
| **Customer Retention** | 70%+ | 80%+ | 85%+ |
| **Repeat Order Rate** | 50%+ | 70%+ | 80%+ |
| **Net Promoter Score (NPS)** | 50+ | 60+ | 70+ |

### **Quality KPIs (AS9100 Required)**
| Metric | Month 12 | Month 24 | Month 36 |
|--------|----------|----------|----------|
| **NCRs (Nonconformance Reports)** | <2/month | <3/month | <5/month |
| **CARs (Corrective Actions)** | <1/month | <2/month | <3/month |
| **Customer Complaints** | <1/quarter | <1/quarter | <1/quarter |
| **Internal Audit Findings** | <10/year | <15/year | <20/year |

---

## 🚧 **Risk Mitigation Plan**

### **Technical Risks**

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **Scrap on Titanium/Inconel** | Medium | High ($500–$2K/part) | - Prototype in Aluminum first<br>- Simulation software (Vericut)<br>- Customer pays material on scrap (negotiate upfront) |
| **Machine Downtime** | Medium | High (lost revenue $2K–$5K/day) | - Preventive maintenance (monthly)<br>- Spare parts inventory ($5K buffer)<br>- Service contract with machine vendor |
| **Certification Audit Failure** | Low | Very High (delays market entry 6+ months) | - Hire experienced consultant<br>- 3 internal audits before certification<br>- Pre-assessment (optional but recommended) |

### **Business Risks**

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **Customer Concentration** | High | High (1 customer = 50%+ revenue = risky) | - Target 10+ active customers<br>- No single customer >25% revenue<br>- Diversify industries (aerospace, medical, defense) |
| **Pricing Pressure** | Medium | Medium (margin compression) | - AS9100 cert = pricing power<br>- Focus on complex parts (not commodity)<br>- Offer value-adds (fast turnaround, DFM) |
| **Supply Chain Delays** | Medium | Medium (material shortages delay delivery) | - Dual-source materials (2+ suppliers per material)<br>- Buffer stock (1-month Aluminum, 2-week Titanium)<br>- Lead time transparency with customers |

### **Financial Risks**

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **Cash Flow Gap (Months 1-6)** | High | High (can't cover operating costs) | - Self-fund Phase 1 from trading bot or savings<br>- Net-30 terms (customers pay fast; you pay suppliers Net-60)<br>- Credit line ($25K–$50K emergency buffer) |
| **Equipment Financing Burden** | Medium | Medium (Phase 3 expansion delayed) | - Don't buy second machine until $100K+/month sustained<br>- Finance at <6% (shop around)<br>- Ensure ROI payback <24 months before committing |

---

## 🎯 **Decision Gates (Go/No-Go Checkpoints)**

### **Gate 1: Month 6 - Portfolio & First Customers**
**Decision Criteria:**
- ✅ 10 portfolio parts built (quality acceptable?)
- ✅ 3+ customer inquiries received
- ✅ 1+ paying job quoted or in progress

**Go**: Proceed to Month 7-12 (customer acquisition, AS9100 prep)  
**No-Go**: Pivot (reconsider market; improve portfolio; address capability gaps)

---

### **Gate 2: Month 12 - AS9100 Commitment**
**Decision Criteria:**
- ✅ 5+ paying customers acquired
- ✅ $10K–$30K revenue achieved (Year 1)
- ✅ 3+ repeat customers (proof of quality/service)
- ✅ Positive customer feedback (NPS 50+)

**Go**: Commit to AS9100 certification (invest $30K–$60K in Phase 2)  
**No-Go**: Stay ISO 9001; focus on commercial work; revisit aerospace in 12 months

---

### **Gate 3: Month 24 - Capacity Expansion**
**Decision Criteria:**
- ✅ AS9100 certified
- ✅ $80K–$150K/month revenue sustained (3+ months)
- ✅ 60–75% machine utilization (current capacity maxed)
- ✅ 5+ aerospace customers with repeat orders

**Go**: Invest in second CNC cell ($300K–$400K)  
**No-Go**: Optimize current capacity (add shifts, outsource overflow); delay expansion 6 months

---

## 📞 **Support Resources**

### **Consultants & Training**
- **AS9100 Consultants**: Search "AS9100 consultant [your state]" or ASQ (American Society for Quality) directory
- **Internal Auditor Training**: ASQ, Exemplar Global, consultant-led (2-day course; $1K–$1.5K/person)
- **GD&T Training**: ASME Y14.5 courses (online or in-person; $500–$1,500)

### **Registrars (Certification Bodies)**
- SAI Global: https://www.saiglobal.com
- NQA: https://www.nqa.com
- Perry Johnson Registrars: https://www.pjr.com
- BSI Group: https://www.bsigroup.com

### **Industry Associations**
- **NDIA** (National Defense Industrial Association): https://www.ndia.org
- **SAE International**: https://www.sae.org
- **OASIS** (Online Aerospace Supplier Information System): https://www.sae.org/iaqg/oasis

### **Material Suppliers (Aerospace-Grade)**
- Aircraft Spruce: https://www.aircraftspruce.com
- Online Metals: https://www.onlinemetals.com
- McMaster-Carr: https://www.mcmaster.com (limited aerospace certs)
- Specialty suppliers: Search "Titanium mill cert supplier [your state]"

### **Software Tools**
- **CAM**: Mastercam, Fusion 360, SolidCAM, Esprit
- **Simulation**: Vericut (CGTech), Mastercam Simulator
- **QMS**: ETQ Reliance, MasterControl, Qualio
- **ERP/MES**: IQMS, Plex, E2 Shop System, JobBOSS

---

## 🔗 **Cross-References**

- **KNOWLEDGE/Business/Manufacturing_Strategy/Precision_Automation_Competitive_Advantage.md** → Strategic foundation
- **KNOWLEDGE/Standards/AS9100_Certification_Guide.md** → Detailed certification requirements
- **PROJECTS/Active/precision_specialty_job_shop/README.md** → Detailed project structure

---

## ✅ **Next Steps: Week 1 Actions**

### **Immediate Actions (This Week)**
1. ✅ **Equipment Inventory**: Photograph shop; document all machines, robots, tooling (4 hours)
2. ✅ **Capability Assessment**: Machine 3 test parts (simple bracket, threaded fitting, complex contour); measure tolerances (8 hours)
3. ✅ **Material Sourcing**: Contact 3 suppliers (Aircraft Spruce, Online Metals, local distributor); request quotes for Al 7075, Ti-6Al-4V, SS 316 (2 hours)
4. ✅ **Consultant Research**: Google "AS9100 consultant [your state]"; request info from 3 consultants (1 hour)
5. ✅ **Financial Planning**: Review cash flow; confirm $40K–$80K available for Phase 1 (or financing plan) (2 hours)

**Total Time**: 17 hours (spread over Week 1)

**Outcome**: Clear picture of current state; ready to start Month 1 portfolio development

---

**Project Status**: Roadmap complete. Ready to execute Month 1 Week 1 actions. Recommend integrating with APM workflow and pushing to GitHub for version control and progress tracking.

**Part of APM Active Projects** | **Strategic Business Development Track**
