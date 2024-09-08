import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from "@/components/ui/card"
import { Label } from "@/components/ui/label"
import { Input } from "@/components/ui/input"
import { Button } from "@/components/ui/button"

export default function paymentSection() {
  return (
    <div className="flex flex-col min-h-[100dvh]">
      <header className="px-5 bg-primary text-primary-foreground py-4">
        <div className="container mx-auto flex justify-between items-center">
          <div className="text-xl font-bold">AURA DIGITAL LABS</div>
        </div>
      </header>
      <div className="flex items-center justify-center p-4 md:p-6 flex-1 opacity-90">
        <Card className="w-full max-w-md">
          <CardHeader>
            <CardTitle>Payment Details</CardTitle>
            <CardDescription>Review and update your payment information.</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="grid gap-1">
              <Label htmlFor="name">Account Name</Label>
              <Input id="name" defaultValue="M R K J M M S B MADUGALLA
" />
            </div>
            <div className="grid gap-1">
              <Label htmlFor="branch">Account Branch</Label>
              <Input id="branch" defaultValue="Wattegama (340)" />
            </div>
            <div className="grid gap-1">
              <Label htmlFor="bank">Bank Name</Label>
              <Input id="bank" defaultValue="Bank of Ceylon " />
            </div>
            <div className="grid gap-1">
              <Label htmlFor="number">Account Number</Label>
              <Input id="number" defaultValue="0086914903" />
            </div>
          </CardContent>
          <CardFooter className="flex justify-end gap-2">
            {/* <Button variant="outline">Cancel</Button> */}
            <Button>Save Payment Details</Button>
          </CardFooter>
        </Card>
      </div>
      <footer className="bg-primary text-primary-foreground py-4">
        <div className="container mx-auto flex justify-center">
          <div className="text-sm">© 2024 AURA DIGITAL LABS. All rights reserved.</div>
        </div>
      </footer>
    </div>
  )
}
