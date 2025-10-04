# ISO 13849 - Safety of Machinery: Functional Safety
*Safety-related parts of control systems*

## Overview
ISO 13849 provides safety requirements and guidance on the principles for the design and integration of safety-related parts of control systems (SRP/CS), including the design of software.

## Performance Levels (PL)

### **Category Architecture Requirements**

#### **Category B** - Basic Category
- Single channel
- Well-tried components and safety principles
- No fault tolerance
- **PL**: a, b

#### **Category 1** - Well-tried components and principles
- Single channel with well-tried components
- Higher reliability than Category B
- **PL**: a, b, c

#### **Category 2** - Single channel with test equipment
- Test equipment detects faults
- Periodic testing by machine control system
- **PL**: a, b, c, d

#### **Category 3** - Dual channel with cross-monitoring ⭐ **OUR TARGET**
- Dual channel architecture
- Cross-monitoring between channels
- Single fault tolerance
- **PL**: a, b, c, d, e
- **Application**: Robot safety systems, emergency stops

#### **Category 4** - Dual channel with fault detection
- Highest safety integrity
- Fault detection and system response
- **PL**: e only

## Implementation for Mini Prototype

### **Category 3 Architecture**
```
SAFETY INPUT DEVICES          LOGIC SOLVER              FINAL ELEMENTS
┌─────────────────┐          ┌─────────────────┐      ┌─────────────────┐
│ E-Stop Button 1 ├──────────┤ Safety PLC Ch A ├──────┤ Contactor A     │
└─────────────────┘          │                 │      └─────────────────┘
┌─────────────────┐          │ Cross-Monitor   │      ┌─────────────────┐
│ E-Stop Button 2 ├──────────┤ Safety PLC Ch B ├──────┤ Contactor B     │
└─────────────────┘          └─────────────────┘      └─────────────────┘
┌─────────────────┐                   │               ┌─────────────────┐
│ Light Curtain   ├───────────────────┼───────────────┤ Motor Power     │
└─────────────────┘                   │               └─────────────────┘
┌─────────────────┐                   ▼               ┌─────────────────┐
│ Guard Switches  ├─────────────> DIAGNOSTIC      ────┤ Status Outputs  │
└─────────────────┘               MONITORING          └─────────────────┘
```

### **Safety Functions Implementation**

#### **Emergency Stop (STO - Safe Torque Off)**
- **Function**: Remove power from all actuators
- **Response Time**: <500ms (typical <100ms)
- **Implementation**: Dual channel safety relays
- **Testing**: Manual test weekly, automatic test daily

#### **Safety-Limited Speed (SLS)**
- **Function**: Limit robot velocity in teaching mode
- **Speed Limit**: 250mm/s maximum in manual mode
- **Monitoring**: Encoder feedback on all axes
- **Override**: Impossible without maintenance key

#### **Safe Operating Stop (SOS)**
- **Function**: Monitor standstill condition
- **Tolerance**: ±5mm position deviation
- **Application**: Door open, maintenance mode
- **Recovery**: Automatic when conditions clear

### **Risk Assessment Matrix**

| Hazard | Severity | Frequency | Avoidance | PLr | Category |
|--------|----------|-----------|-----------|-----|----------|
| Robot collision | S2 | F2 | P2 | **d** | **3** |
| Crushing/pinching | S2 | F1 | P1 | **c** | **2** |
| Pneumatic release | S1 | F2 | P2 | **b** | **1** |
| Electrical shock | S2 | F1 | P1 | **c** | **2** |

### **Validation Requirements**

#### **Design Verification**
- [ ] FMEA (Failure Mode Effects Analysis) complete
- [ ] FTA (Fault Tree Analysis) documented
- [ ] Safety calculations validated
- [ ] Software verification per ISO 13849-2

#### **Installation Validation**
- [ ] Safety circuit continuity testing
- [ ] Response time measurement
- [ ] Fault injection testing
- [ ] Documentation review and approval

### **Maintenance & Testing**

#### **Daily Checks**
- Emergency stop function test
- Light curtain beam interruption test
- Guard door interlock verification
- Visual inspection of safety devices

#### **Weekly Checks**
- Safety relay contact verification
- Pneumatic pressure safety valve test
- Cable and connection inspection
- Safety system diagnostic review

#### **Annual Verification**
- Complete safety function testing
- Response time measurement
- Documentation update
- Third-party safety audit

## Compliance Documentation

### **Required Documents**
1. **Safety Requirements Specification (SRS)**
2. **Safety Validation Plan**
3. **FMEA and Risk Assessment**
4. **Safety Manual and Operating Procedures**
5. **Maintenance and Testing Protocols**

### **Certification Process**
- Design review by certified functional safety engineer
- Third-party validation (TÜV, UL, CSA)
- Installation inspection and commissioning
- Annual compliance verification

*This Category 3 implementation provides single fault tolerance with cross-monitoring, ensuring safe operation of the robotic manufacturing system.*