export default function FooterSection() {
  return (
    <footer className="bg-gray-100 py-8">
      <div className="container mx-auto px-4">
        <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-8">
          <div>
            <h2 className="text-lg font-bold mb-4">Company</h2>
            <ul className="space-y-2">
              <li>About Us</li>
              <li>Our Team</li>
              <li>Careers</li>
              <li>Press</li>
              <li>Blog</li>
            </ul>
          </div>
          <div>
            <h2 className="text-lg font-bold mb-4">Services</h2>
            <ul className="space-y-2">
              <li>Product Development</li>
              <li>Consulting</li>
              <li>Support</li>
              <li>Partnerships</li>
              <li>Client Resources</li>
            </ul>
          </div>
          <div>
            <h2 className="text-lg font-bold mb-4">Resources</h2>
            <ul className="space-y-2">
              <li>Documentation</li>
              <li>Tutorials</li>
              <li>FAQs</li>
              <li>Community Forum</li>
              <li>Case Studies</li>
            </ul>
          </div>
          <div>
            <h2 className="text-lg font-bold mb-4">Contact</h2>
            <ul className="space-y-2">
              <li>Contact Us</li>
              <li>Support Center</li>
              <li>Request a Demo</li>
              <li>Feedback</li>
              <li>Social Media</li>
            </ul>
          </div>
        </div>
        <div className="mt-8 text-center text-sm text-gray-600">
          <p>
            Aura Digital Labs is dedicated to delivering innovative solutions
            and exceptional service. Stay connected with us to explore how we
            can help you achieve your goals.
          </p>
        </div>
      </div>
    </footer>
  );
}
