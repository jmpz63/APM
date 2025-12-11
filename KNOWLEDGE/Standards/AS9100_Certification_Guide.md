# AS9100 Aerospace Quality Management System - Certification Guide

**Standard**: AS9100 Revision D (current as of 2025)  
**Based On**: ISO 9001:2015 + Aerospace-Specific Requirements  
**Applies To**: Aviation, Space, and Defense Organizations  
**Scope**: Design, Development, Production, Installation, and Servicing of Aerospace Products  
**Date Created**: 2025-12-11  

---

## 📋 **Overview**

### **What is AS9100?**
AS9100 is a **quality management system (QMS) standard** specifically designed for the aerospace industry. It builds upon ISO 9001:2015 by adding aerospace-specific requirements for:
- Configuration management
- Risk management (DFMEA/PFMEA)
- First Article Inspection (FAI)
- Product safety and airworthiness
- Counterfeit parts prevention
- On-time delivery performance

### **Why AS9100 Matters**
- **Market Access**: Required by Boeing, Airbus, Lockheed Martin, Northrop Grumman, and most Tier-1 aerospace suppliers
- **Competitive Advantage**: Certified shops command 2–4× higher pricing than non-certified competitors
- **Risk Mitigation**: Structured approach to quality reduces defects, scrap, and customer complaints
- **Continuous Improvement**: Built-in feedback loops drive operational excellence

### **Who Needs AS9100?**
- **Tier-1 Suppliers**: Direct suppliers to OEMs (Boeing, Airbus, etc.)
- **Tier-2/Tier-3 Suppliers**: Sub-tier suppliers to Tier-1 companies
- **Contract Manufacturers**: Job shops machining aerospace parts
- **MRO (Maintenance, Repair, Overhaul)**: Facilities servicing aerospace equipment

---

## 🎯 **AS9100 vs ISO 9001: Key Differences**

| Requirement | ISO 9001:2015 | AS9100 Rev D (Adds) |
|-------------|---------------|---------------------|
| **Configuration Management** | Not required | Mandatory (control of design changes, part revisions, customer approvals) |
| **Risk Management** | Context of organization | DFMEA (Design) + PFMEA (Process) required |
| **First Article Inspection (FAI)** | Not specified | AS9102 forms required for new parts/processes |
| **Product Safety** | General quality | Explicit product safety and airworthiness focus |
| **On-Time Delivery** | Not specified | KPI required; must track and report OTD performance |
| **Counterfeit Parts Prevention** | Not specified | Traceability and supplier controls to prevent counterfeit/suspect parts |
| **Special Processes** | General control | Enhanced control (welding, heat treat, NDT, plating) |
| **Key Characteristics** | Not specified | Critical features must be identified and controlled with statistical methods |

---

## 📝 **AS9100 Requirements (10 Clauses)**

### **Clause 1–3: Introduction, Normative References, Terms & Definitions**
*(Informational; not auditable)*

---

### **Clause 4: Context of the Organization**

#### **4.1 Understanding the Organization and Its Context**
- Identify internal/external issues affecting QMS (market trends, competition, regulations)

#### **4.2 Understanding the Needs and Expectations of Interested Parties**
- **Interested Parties**: Customers, regulators (FAA, EASA), suppliers, employees
- Document requirements and expectations

#### **4.3 Determining the Scope of the QMS**
- Define what the QMS covers (e.g., "CNC machining of aerospace brackets and fittings")
- Exclusions must be justified (e.g., "Design excluded; customer provides drawings")

#### **4.4 Quality Management System and Its Processes**
- **Process Map**: Document key processes (quoting, production, inspection, shipping)
- **Turtle Diagrams**: For each process, define inputs, outputs, resources, controls, metrics

**AS9100-Specific Additions:**
- **Product Safety**: Identify processes that affect product safety (e.g., heat treatment, welding)
- **Risk-Based Thinking**: Risk assessment for each process

---

### **Clause 5: Leadership**

#### **5.1 Leadership and Commitment**
- Top management (owner/CEO) must demonstrate commitment to QMS
- **Evidence**: Management review meetings, resource allocation, quality policy communication

#### **5.2 Quality Policy**
- Written statement of quality intent
- **Example**: "ABC Machining is committed to delivering aerospace components that meet customer specifications, regulatory requirements, and AS9100 standards through continuous improvement."

#### **5.3 Organizational Roles, Responsibilities, and Authorities**
- **Org Chart**: Define who does what (Quality Manager, Production Manager, etc.)
- **Quality Manager**: Must have authority independent of production (can stop shipment if quality issue)

**AS9100-Specific Additions:**
- **Product Safety Manager**: If applicable (for safety-critical parts)
- **Configuration Manager**: Control of engineering changes

---

### **Clause 6: Planning**

#### **6.1 Actions to Address Risks and Opportunities**
- **Risk Assessment**: Identify risks to quality (e.g., supplier delays, machine breakdowns, skill gaps)
- **Opportunities**: Cost reduction, new markets, technology upgrades

**AS9100-Specific Additions:**
- **DFMEA (Design Failure Mode Effects Analysis)**: If you design parts
- **PFMEA (Process FMEA)**: For manufacturing processes (identify failure modes, severity, occurrence, detection)

#### **6.2 Quality Objectives and Planning to Achieve Them**
- **SMART Goals**: Specific, Measurable, Achievable, Relevant, Time-bound
- **Examples**:
  - "Achieve 98% on-time delivery by Q4 2026"
  - "Reduce scrap rate to <3% by Q2 2026"
  - "Obtain AS9100 certification by December 2026"

#### **6.3 Planning of Changes**
- When changes occur (new equipment, new process, staff turnover), plan and control the transition

---

### **Clause 7: Support**

#### **7.1 Resources**
- **Infrastructure**: Machines, tools, facilities, IT systems
- **Work Environment**: Temperature/humidity control (if required), cleanliness, lighting
- **Monitoring and Measuring Resources**: Calibrated equipment (calipers, micrometers, CMM)

**AS9100-Specific Additions:**
- **Calibration**: All measurement equipment must have traceable calibration (NIST or equivalent)
- **Maintenance**: Preventive maintenance schedule for production equipment

#### **7.2 Competence**
- **Training Matrix**: Define required skills for each role (CNC programmer, inspector, operator)
- **Training Records**: Document training completion dates, certifications

#### **7.3 Awareness**
- Employees must understand:
  - Quality policy and objectives
  - Their contribution to QMS effectiveness
  - Consequences of nonconformance (e.g., aircraft safety)

#### **7.4 Communication**
- Internal communication (management meetings, shift handoffs)
- External communication (customer notifications, supplier coordination)

#### **7.5 Documented Information**
- **Quality Manual**: High-level QMS overview (20–40 pages)
- **Procedures**: Detailed "how-to" documents (20–30 procedures typical)
- **Work Instructions**: Step-by-step job aids (as needed for complex tasks)
- **Forms/Records**: Inspection reports, calibration logs, training records, NCRs (Nonconformance Reports)

**AS9100-Specific Additions:**
- **Configuration Management**: Control of engineering drawings, revisions, approvals
- **Data Retention**: Records must be kept for 10+ years (or per customer contract)

---

### **Clause 8: Operation**

#### **8.1 Operational Planning and Control**
- **Production Planning**: Schedule, resource allocation, risk mitigation
- **Outsourcing**: Control of subcontracted processes (e.g., heat treatment, plating)

**AS9100-Specific Additions:**
- **Product Safety Plan**: For safety-critical parts (e.g., landing gear components)
- **Operational Risk Management**: PFMEA for key processes

#### **8.2 Requirements for Products and Services**

**8.2.1 Customer Communication**
- Clarify customer requirements (drawings, specs, delivery dates)
- Handle inquiries, quotes, and order confirmations

**8.2.2 Determining Requirements**
- Technical requirements (dimensions, tolerances, materials)
- Regulatory requirements (FAA, EASA, military specs)
- Customer-specific requirements (AS9102 FAI, special packaging)

**8.2.3 Review of Requirements**
- **Contract Review**: Before accepting order, verify you can meet all requirements
- **Evidence**: Signed contract review checklist or meeting minutes

**AS9100-Specific Additions:**
- **Customer-Furnished Property**: Control of customer-provided materials, tooling, or data
- **Special Requirements**: Identify key characteristics (critical dimensions, critical processes)

#### **8.3 Design and Development** *(Skip if "Design Excluded" in Scope)*
*(If applicable, covers design inputs, outputs, verification, validation, changes)*

#### **8.4 Control of Externally Provided Processes, Products, and Services**
- **Approved Supplier List (ASL)**: Only buy from qualified suppliers
- **Supplier Evaluation**: Criteria: quality, delivery, price, certifications (ISO 9001, AS9100, Nadcap)
- **Supplier Audits**: On-site audits for critical suppliers (annually or per risk assessment)
- **Purchase Orders**: Include technical requirements, material certs, traceability requirements

**AS9100-Specific Additions:**
- **Counterfeit Parts Prevention**: Verify authenticity of materials (mill certs, supplier reputation)
- **Right of Access**: Customer or regulator can audit your suppliers (must be in PO terms)

#### **8.5 Production and Service Provision**

**8.5.1 Control of Production**
- **Work Instructions**: For complex operations (e.g., 5-axis machining setup)
- **Process Control**: Monitor key parameters (feeds, speeds, coolant flow)
- **Equipment Maintenance**: Preventive maintenance schedule; document completion

**AS9100-Specific Additions:**
- **Production Process Verification**: First article, in-process inspection, final inspection
- **Control of Production Equipment, Tools, and Software**: Calibration, validation, version control
- **Post-Delivery Support**: Spare parts, service manuals (if applicable)

**8.5.2 Identification and Traceability**
- **Lot/Serial Tracking**: Each part must be traceable to:
  - Raw material (heat lot, mill cert)
  - Manufacturing date
  - Inspector
  - Customer order

**AS9100-Specific Additions:**
- **Full Traceability**: From raw material to finished part to customer delivery
- **Serialization**: If required by customer (common for high-value parts)

**8.5.3 Property Belonging to Customers or External Providers**
- Control of customer-furnished materials, tooling, or intellectual property
- **Example**: Customer provides titanium billet; you must track, protect, and return scrap/excess

**8.5.4 Preservation**
- Protect parts during production, storage, and shipping (avoid contamination, corrosion, damage)
- **Packaging**: Use anti-corrosion packaging for metals; foam inserts for delicate parts

**8.5.5 Post-Delivery Activities**
- Warranty, service, spare parts (if applicable)

**8.5.6 Control of Changes**
- Engineering changes must be reviewed and approved before implementation
- **Change Control Board**: Review changes for impact on quality, cost, schedule

**AS9100-Specific Additions:**
- **Configuration Management**: All changes documented in revision history; customer approval if required
- **FAI for Changes**: If process or design changes, new First Article Inspection may be required

#### **8.6 Release of Products and Services**
- **Final Inspection**: Verify all requirements met before shipment
- **Inspection Records**: Document dimensions, material certs, visual inspection, FAI (if applicable)
- **Release Authority**: Designated person (Quality Manager or inspector) authorizes release

**AS9100-Specific Additions:**
- **First Article Inspection (FAI)**: AS9102 forms required for:
  - New parts
  - Engineering changes
  - Process changes
  - Tooling changes
  - After 2+ years of no production

#### **8.7 Control of Nonconforming Outputs**
- **Nonconformance Report (NCR)**: Document defects (out-of-tolerance, wrong material, etc.)
- **Disposition**:
  - **Rework**: Fix and re-inspect
  - **Scrap**: Destroy (document destruction)
  - **Use-As-Is**: Customer approval required (rare in aerospace)
  - **Return to Supplier**: If supplier-caused defect
- **Root Cause Analysis**: For recurring issues (5 Whys, Fishbone Diagram, 8D)

**AS9100-Specific Additions:**
- **Customer Notification**: Notify customer of nonconformance if:
  - Part already shipped
  - Affects product safety or airworthiness
  - Customer contract requires notification

---

### **Clause 9: Performance Evaluation**

#### **9.1 Monitoring, Measurement, Analysis, and Evaluation**

**9.1.1 General**
- Define metrics (KPIs) to track QMS performance

**9.1.2 Customer Satisfaction**
- Surveys, scorecards, complaint tracking
- **AS9100 Requirement**: On-time delivery (OTD) performance must be tracked

**9.1.3 Analysis and Evaluation**
- Analyze data trends (scrap rate, OTD, customer complaints)
- Use data for decision-making

**AS9100-Specific Additions:**
- **OTD Performance**: Track % of orders delivered on or before promised date (target: 95%+)
- **Key Characteristics**: Statistical process control (SPC) for critical dimensions

#### **9.2 Internal Audit**
- **Frequency**: At least annually (more often for large QMS)
- **Scope**: All processes and all shifts audited within each cycle
- **Auditors**: Trained internal auditors (can be employees; must be independent of audited area)
- **Audit Report**: Findings, nonconformances, opportunities for improvement
- **Corrective Actions**: Address nonconformances within defined timeframe

#### **9.3 Management Review**
- **Frequency**: At least annually (quarterly recommended)
- **Attendees**: Top management + key staff (Quality Manager, Production Manager, etc.)
- **Inputs**:
  - Internal audit results
  - Customer feedback
  - Process performance (KPIs)
  - Nonconformances and corrective actions
  - Changes in external/internal issues
  - Resource needs
- **Outputs**:
  - Opportunities for improvement
  - Changes to QMS
  - Resource decisions

---

### **Clause 10: Improvement**

#### **10.1 General**
- Continually improve QMS effectiveness
- **Methods**: Kaizen, Lean, Six Sigma, corrective actions

#### **10.2 Nonconformity and Corrective Action (CAPA)**
- **When Nonconformance Occurs**:
  1. **React**: Contain the problem (quarantine defective parts)
  2. **Investigate**: Root cause analysis (5 Whys, Fishbone, 8D)
  3. **Implement Corrective Action**: Fix the root cause (not just the symptom)
  4. **Verify Effectiveness**: Monitor to ensure issue doesn't recur
  5. **Update QMS**: Revise procedures if needed

**AS9100-Specific Additions:**
- **Customer Corrective Action Requests (SCAR, 8D)**: Respond to customer-initiated CARs within required timeframe (typically 30 days)

#### **10.3 Continual Improvement**
- Use data, audits, and management review to drive improvement
- **Examples**: Reduce cycle time, improve first-pass yield, automate inspection

---

## 🚀 **AS9100 Certification Process**

### **Step 1: Gap Analysis (Month 1–2)**

**Objective**: Identify gaps between current practices and AS9100 requirements

**Activities:**
1. **Hire Consultant** (optional but highly recommended; $10K–$20K for 6–12 months)
   - Look for consultants with aerospace experience + AS9100 Lead Auditor training
2. **Review Current Processes**: Document what you currently do (even if informal)
3. **Compare to AS9100 Requirements**: Identify missing procedures, records, or controls
4. **Prioritize Gaps**: Focus on high-impact areas first (traceability, calibration, FAI)

**Deliverable**: Gap analysis report with action plan

---

### **Step 2: QMS Development (Month 3–9)**

**Objective**: Build documented QMS (Quality Manual, procedures, work instructions, forms)

**Quality Manual (20–40 pages)**
- Scope of QMS
- Quality policy
- Organizational chart
- Process map
- Reference to procedures

**Procedures (20–30 documents typical)**
| Procedure | Purpose |
|-----------|---------|
| **Document Control** | How to create, approve, revise, and archive documents |
| **Record Control** | How to identify, store, protect, and dispose of records |
| **Management Review** | Frequency, agenda, inputs, outputs |
| **Internal Audit** | Audit planning, execution, reporting |
| **Corrective Action (CAPA)** | NCR process, root cause analysis, verification |
| **Preventive Action** | Risk-based thinking, proactive improvements |
| **Control of Nonconforming Product** | Quarantine, disposition, customer notification |
| **Calibration** | Calibration intervals, traceable standards, out-of-tolerance actions |
| **Training** | Competence requirements, training records |
| **Supplier Management** | Approved supplier list, evaluation, audits, PO requirements |
| **Production Control** | Work instructions, process monitoring, equipment maintenance |
| **Inspection & Testing** | Receiving inspection, in-process inspection, final inspection, FAI |
| **Traceability** | Lot tracking, serialization, material certs |
| **Configuration Management** | Engineering change control, revision history |
| **Customer Communication** | Order entry, contract review, complaint handling |

**Work Instructions (as needed)**
- CNC setup procedures
- Inspection procedures (how to use CMM, height gauge, etc.)
- Packaging/shipping procedures

**Forms/Templates**
- Nonconformance Report (NCR)
- Corrective Action Request (CAR)
- First Article Inspection Report (AS9102 Form 1, 2, 3)
- Calibration log
- Training matrix
- Internal audit checklist
- Management review agenda
- Job traveler (route sheet with inspection checkpoints)

**Software Tools (Optional but Helpful)**
- **QMS Software**: ETQ Reliance, MasterControl, Qualio (automates document control, CAPA, audits)
- **ERP/MES**: For production tracking, traceability (e.g., IQMS, Plex, E2 Shop System)

---

### **Step 3: Implementation & Training (Month 6–10)**

**Objective**: Put QMS into practice; train staff

**Activities:**
1. **Roll Out Procedures**: Start using documented processes in daily operations
2. **Training**: Train all employees on:
   - Quality policy and objectives
   - Their role in the QMS
   - How to use forms (NCR, CAPA, job travelers)
3. **Practice Traceability**: Run pilot parts with full lot tracking, material certs, FAI
4. **Calibration**: Ensure all measurement equipment calibrated; maintain calibration log
5. **Management Review**: Hold first management review meeting; document minutes

**Timeline**: 3–6 months (while developing procedures)

---

### **Step 4: Internal Audits (Month 10–12)**

**Objective**: Verify QMS is working; identify issues before certification audit

**Activities:**
1. **Train Internal Auditors**: 2–3 employees complete AS9100 Internal Auditor training (2-day course; ~$1,000/person)
2. **Conduct Internal Audits**:
   - Audit all processes (production, inspection, calibration, document control, CAPA, etc.)
   - Use AS9100 checklist (available from consultants or registrars)
3. **Document Findings**:
   - Nonconformances (NCs): Violations of AS9100 or your procedures
   - Observations: Opportunities for improvement
4. **Corrective Actions**: Fix NCs; verify effectiveness
5. **Audit Report**: Present to management review

**Frequency**: At least one full internal audit before certification audit

---

### **Step 5: Pre-Assessment (Optional; Month 13–14)**

**Objective**: Dry run with registrar before official certification audit

**Activities:**
1. **Contact Registrar**: Choose AS9100 registrar (e.g., SAI Global, NQA, Perry Johnson, BSI)
2. **Pre-Assessment Audit**: Registrar conducts 1-day on-site audit
   - Informal; identifies gaps
   - Not a pass/fail; purely advisory
3. **Address Gaps**: Fix issues identified in pre-assessment

**Cost**: $2,000–$5,000 (optional; can skip if confident in QMS readiness)

---

### **Step 6: Certification Audit (Month 15–18)**

**Objective**: Official AS9100 certification audit by accredited registrar

**Stage 1 Audit (Document Review)**
- **Duration**: 1 day (remote or on-site)
- **Focus**: Review Quality Manual, procedures, records
- **Outcome**: "Ready for Stage 2" or "Address deficiencies"

**Stage 2 Audit (On-Site Implementation Audit)**
- **Duration**: 2–3 days (depends on company size and scope)
- **Activities**:
  - Interview employees (Do they know the QMS? Follow procedures?)
  - Observe operations (CNC machining, inspection, calibration)
  - Review records (job travelers, NCRs, CARs, training records, calibration logs)
  - Verify traceability (Can you trace a finished part back to raw material cert?)
- **Findings**:
  - **Major Nonconformances**: Critical gaps (e.g., no calibration, no traceability). Must be fixed before certification.
  - **Minor Nonconformances**: Small issues (e.g., missing signature on one form). Can be fixed within 90 days.
  - **Observations**: Opportunities for improvement (no action required).

**Post-Audit Corrective Actions**
- Fix major NCs immediately (within 30 days typical)
- Provide evidence to registrar (revised procedures, photos, records)
- Registrar reviews evidence; if satisfied, issues certificate

**Certification Issued**
- Valid for 3 years
- **Surveillance Audits**: Annual 1-day audits to verify ongoing compliance

**Cost**: $10,000–$25,000 (initial certification) + $3,000–$8,000/year (surveillance)

---

## 📅 **Timeline Summary**

| Month | Activity | Cost |
|-------|----------|------|
| **1–2** | Gap analysis; hire consultant | $2K–$5K (consultant retainer) |
| **3–9** | QMS development (manual, procedures, forms) | $8K–$15K (consultant fees) |
| **6–10** | Implementation & training | $2K–$5K (training courses) |
| **10–12** | Internal audits | $1K–$2K (auditor training) |
| **13–14** | Pre-assessment (optional) | $2K–$5K |
| **15–18** | Certification audit (Stage 1 + Stage 2) | $10K–$25K |
| **Ongoing** | Surveillance audits (annual) | $3K–$8K/year |

**Total Investment**: $25,000–$60,000 over 18 months

---

## ✅ **AS9100 Certification Checklist**

### **Pre-Audit Preparation**

#### **Documentation Complete**
- [ ] Quality Manual (scope, policy, process map, procedures list)
- [ ] 20–30 Procedures (document control, CAPA, calibration, production, inspection, etc.)
- [ ] Work Instructions (as needed for complex tasks)
- [ ] Forms/Templates (NCR, CAR, AS9102 FAI, job travelers, calibration log)

#### **Records Available (3–6 months of history minimum)**
- [ ] Job travelers with inspection checkpoints (10+ jobs)
- [ ] First Article Inspection Reports (AS9102; 3–5 samples)
- [ ] Nonconformance Reports (NCRs; 3–5 samples with root cause and corrective action)
- [ ] Calibration records (all measurement equipment; current certs)
- [ ] Training records (all employees; competence matrix)
- [ ] Internal audit reports (at least 1 full audit)
- [ ] Management review minutes (at least 1 meeting)
- [ ] Supplier evaluations (approved supplier list; 3–5 suppliers evaluated)

#### **Physical Evidence**
- [ ] Calibration stickers on all measurement equipment (valid dates)
- [ ] Job travelers accompany work-in-progress (WIP) on shop floor
- [ ] Nonconforming material segregated (quarantine area or red tags)
- [ ] Material certs filed and accessible (mill certs for raw material)
- [ ] Org chart posted or available (roles and responsibilities clear)

#### **Employee Readiness**
- [ ] All employees trained on quality policy and their QMS role
- [ ] Employees can explain their job and how it affects quality
- [ ] Employees know where to find procedures and work instructions
- [ ] Employees know how to report nonconformances (NCR process)

---

## 🎯 **Key Success Factors**

### **Top Management Commitment**
- Owner/CEO must actively support QMS (not just delegate to Quality Manager)
- Provide resources: time, budget, staff
- Participate in management reviews and audits

### **Employee Buy-In**
- Explain "why" (not just "what"): QMS helps us win aerospace contracts and avoid mistakes
- Involve employees in procedure development (frontline workers know the job best)
- Recognize good practices (catch people doing things right)

### **Start Simple, Improve Over Time**
- Version 1.0 of procedures doesn't have to be perfect
- Get certified, then refine QMS based on audit feedback and operational experience

### **Leverage Automation**
- Your CNC/robot system IS your process control
- Digital work instructions, traceability databases, and automated data collection simplify compliance

### **Choose the Right Consultant**
- Look for aerospace industry experience (not just generic ISO 9001)
- Ask for references from small job shops (not just large OEMs)
- Consultant should teach you, not do it for you (you need to own the QMS)

---

## 🔗 **Cross-References**

- **PROJECTS/Active/precision_specialty_job_shop/README.md** → Active project using AS9100
- **KNOWLEDGE/Business/Manufacturing_Strategy/Precision_Automation_Competitive_Advantage.md** → Strategic business case
- **KNOWLEDGE/Standards/ISO_13485_Medical_Device_QMS.md** → Alternative certification for medical device market
- **KNOWLEDGE/Standards/ITAR_Compliance_Framework.md** → Defense market requirements

---

## 📚 **Additional Resources**

### **Standards & Documents**
- **AS9100 Rev D Standard**: Purchase from SAE International ($100–$150)
- **AS9102 (FAI Standard)**: Free download from SAE or IAQG
- **ISO 9001:2015**: Foundation standard (AS9100 builds on this)

### **Registrars (Accredited Certification Bodies)**
- SAI Global: https://www.saiglobal.com
- NQA: https://www.nqa.com
- Perry Johnson Registrars (PJR): https://www.pjr.com
- BSI Group: https://www.bsigroup.com

### **Training & Tools**
- **ASQ (American Society for Quality)**: AS9100 training courses, auditor certification
- **Quality Management Systems Academy**: Online AS9100 courses
- **IAQG (International Aerospace Quality Group)**: Free resources, OASIS database (supplier directory)

### **Industry Associations**
- **NADCAP (National Aerospace and Defense Contractors Accreditation Program)**: Accreditation for special processes (heat treat, welding, NDT)
- **SAE International**: Aerospace standards development organization

---

**Summary**: AS9100 certification takes 12–18 months and $25K–$60K but unlocks access to aerospace OEMs paying 2–4× premium rates. Focus on traceability, calibration, FAI, and configuration management. Start with gap analysis, build QMS documentation, implement for 3–6 months, internal audit, then pursue certification. Choose registrar, complete Stage 1/2 audits, address findings, and receive certification valid for 3 years with annual surveillance.

**Date Created**: 2025-12-11  
**Last Updated**: 2025-12-11  
**Tags**: #AS9100 #Aerospace #QualityManagement #Certification #ISO9001 #FAI #Traceability
