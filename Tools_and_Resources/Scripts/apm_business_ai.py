#!/usr/bin/env python3
"""
APM Business Intelligence Automation System
Example: Generate contractor lists and bids from knowledge base

Usage Examples:
    python3 apm_business_ai.py --list-painters --rating 4+
    python3 apm_business_ai.py --generate-bid drywall --sqft 2500 --client "ABC Corp"
    python3 apm_business_ai.py --email-contractors electrical --location "within 25 miles"
"""

import json
import os
from datetime import datetime
from pathlib import Path

class APMBusinessAI:
    def __init__(self, apm_path="/home/arm1/APM"):
        self.apm_path = Path(apm_path)
        self.business_path = self.apm_path / "Business"
        self.contacts_path = self.business_path / "Contacts_Network"
        self.templates_path = self.business_path / "Templates_Documents"
        
    def find_contractors_by_trade(self, trade, min_rating=0, location=None):
        """
        Example: 'Find all painters with 4+ star ratings'
        Returns filtered contractor list
        """
        contractors = []
        trade_path = self.contacts_path / "Contractors" / f"{trade.title()}s"
        
        if not trade_path.exists():
            return f"No {trade} contractors found in database"
        
        # Scan contractor files
        for contractor_file in trade_path.glob("*.md"):
            contractor_data = self._parse_contractor_file(contractor_file)
            
            if contractor_data['rating'] >= min_rating:
                contractors.append(contractor_data)
        
        # Sort by rating, then by recent activity
        contractors.sort(key=lambda x: (x['rating'], x['last_contact']), reverse=True)
        
        return self._format_contractor_list(contractors, trade)
    
    def generate_bid(self, service_type, **parameters):
        """
        Example: Generate drywall bid for 2500 sq ft commercial space
        """
        template_file = self.templates_path / f"{service_type}_bid_template.md"
        
        if not template_file.exists():
            return f"No template found for {service_type} bids"
        
        # Load template
        with open(template_file, 'r') as f:
            template = f.read()
        
        # Auto-populate based on parameters
        populated_bid = self._populate_bid_template(template, service_type, parameters)
        
        # Save generated bid
        client_name = parameters.get('client', 'Unknown_Client')
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bid_filename = f"{service_type}_bid_{client_name}_{timestamp}.md"
        bid_path = self.business_path / "Bids_Proposals" / service_type.title() / bid_filename
        
        # Ensure directory exists
        bid_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(bid_path, 'w') as f:
            f.write(populated_bid)
        
        return f"Bid generated: {bid_path}"
    
    def _populate_bid_template(self, template, service_type, params):
        """
        Auto-populate bid template with calculated values
        """
        if service_type == "drywall":
            return self._populate_drywall_bid(template, params)
        elif service_type == "concrete":
            return self._populate_concrete_bid(template, params)
        # Add more service types as needed
        
        return template
    
    def _populate_drywall_bid(self, template, params):
        """
        Auto-calculate drywall bid costs and populate template
        """
        sqft = params.get('sqft', 0)
        client = params.get('client', '[Client Name]')
        project = params.get('project', '[Project Name]')
        
        # Cost calculations (these would come from your database)
        material_cost_per_sqft = 1.25  # From historical data
        labor_cost_per_sqft = 2.50     # From historical data
        
        materials_total = sqft * material_cost_per_sqft
        labor_total = sqft * labor_cost_per_sqft
        subtotal = materials_total + labor_total
        tax_rate = 0.08  # 8% tax
        tax_amount = subtotal * tax_rate
        total_cost = subtotal + tax_amount
        
        # Timeline calculation (from your experience database)
        days_per_1000_sqft = 4  # Historical average
        timeline_days = max(1, int((sqft / 1000) * days_per_1000_sqft))
        
        # Populate template variables
        replacements = {
            '[Client Name]': client,
            '[Project Name/Address]': project,
            '[Current Date]': datetime.now().strftime("%B %d, %Y"),
            '[Date + 30 days]': (datetime.now().replace(day=datetime.now().day + 30)).strftime("%B %d, %Y"),
            '[X,XXX]': f"{sqft:,}",
            '[X] working days': f"{timeline_days} working days",
            '$[X,XXX]': f"${materials_total:,.0f}",  # Materials
            '$[XX,XXX]': f"${total_cost:,.0f}",      # Total
        }
        
        populated_template = template
        for placeholder, value in replacements.items():
            populated_template = populated_template.replace(placeholder, value)
        
        return populated_template
    
    def _parse_contractor_file(self, filepath):
        """
        Parse contractor markdown file and extract key information
        """
        with open(filepath, 'r') as f:
            content = f.read()
        
        # Simple parsing (in real implementation, use proper markdown parser)
        contractor_data = {
            'name': 'Unknown',
            'company': 'Unknown',
            'rating': 0,
            'phone': 'Unknown',
            'email': 'Unknown',
            'specialties': [],
            'last_contact': '1900-01-01'
        }
        
        # Extract rating (count stars)
        if '⭐' in content:
            rating_line = [line for line in content.split('\n') if '⭐' in line]
            if rating_line:
                contractor_data['rating'] = rating_line[0].count('⭐')
        
        # In real implementation, parse all fields from markdown
        return contractor_data
    
    def _format_contractor_list(self, contractors, trade):
        """
        Format contractor list for easy reading
        """
        if not contractors:
            return f"No {trade} contractors found matching criteria"
        
        output = f"\n📋 **{trade.title()} Contractors Found: {len(contractors)}**\n\n"
        
        for i, contractor in enumerate(contractors, 1):
            stars = '⭐' * contractor['rating']
            output += f"{i}. **{contractor['name']}** - {contractor['company']}\n"
            output += f"   Rating: {stars} ({contractor['rating']}/5)\n"
            output += f"   Phone: {contractor['phone']}\n"
            output += f"   Email: {contractor['email']}\n"
            output += f"   Last Contact: {contractor['last_contact']}\n\n"
        
        return output

# Example usage functions that would be called by AI assistant
def list_painters_4plus_stars():
    """Example: 'Email me painters with 4+ star ratings'"""
    apm = APMBusinessAI()
    return apm.find_contractors_by_trade("painter", min_rating=4)

def generate_drywall_bid_2500sqft(client_name="ABC Corporation"):
    """Example: 'Create a drywall bid for 2,500 sq ft commercial space'"""
    apm = APMBusinessAI()
    return apm.generate_bid("drywall", 
                           sqft=2500, 
                           client=client_name,
                           project="Commercial Office Build-out")

def find_concrete_contractors_commercial():
    """Example: 'Find concrete contractors with commercial experience'"""
    apm = APMBusinessAI()
    return apm.find_contractors_by_trade("concrete", min_rating=3)

# Command line interface
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "--demo":
            print("🚀 APM Business AI Demo")
            print("=" * 50)
            
            print("\n1. Finding painters with 4+ stars...")
            print(list_painters_4plus_stars())
            
            print("\n2. Generating drywall bid...")
            result = generate_drywall_bid_2500sqft("Demo Client Corp")
            print(result)
            
            print("\n3. Finding concrete contractors...")
            print(find_concrete_contractors_commercial())
            
        else:
            print("Available commands: --demo")
    else:
        print(__doc__)