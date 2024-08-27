'use client';
import emailjs from "@emailjs/browser";
import React, { useRef } from "react";
import {MailIcon, MapPinIcon, PhoneIcon } from "@/components/Icons/icons"

export default function EmailingSection() {
    const form = useRef<HTMLFormElement>(null);

    const sendEmail = (e: any) => {
        e.preventDefault();
    
        if (!form.current) {
          return;
        }
        emailjs
          .sendForm(
            "service_2kuohha",
            "template_13tl8ne",
            form.current,
            "T6okvmv1rncTmho2P"
          )
          .then(
            (result) => {
              console.log("SUCCESS!", result.text);
              alert("Message sent successfully");
            },
            (error) => {
              console.log("FAILED...", error.text);
              alert("Failed to send message");
            }
          );
      };
      return(
        <section id="contact" className="w-full py-12 md:py-24 lg:py-32 bg-muted">
        <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6 mx-auto">
          <div className="space-y-3">
            <h2 className="text-3xl font-bold tracking-tighter md:text-4xl/tight">
              Get in Touch
            </h2>
            <p className="mx-auto max-w-[600px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Have a project in mind or need our expertise? Fill out the form
              below and we&apos;ll get back to you as soon as possible.
            </p>
          </div>
          <div className="mx-auto w-full max-w-sm space-y-4">
            <form ref={form} onSubmit={sendEmail} className="grid gap-4">
              <input
                type="text"
                name="user_name"
                placeholder="Name"
                className="max-w-lg flex-1 p-2 rounded-sm border-3 shadow-sm"
                required
              />
              <input
                type="email"
                name="user_email"
                placeholder="Email"
                className="max-w-lg flex-1 p-2 rounded-sm border-3 shadow-sm"
                required
              />
              <textarea
                name="message"
                placeholder="Message"
                className="max-w-lg flex-1 p-2 rounded-sm border-3 shadow-sm"
                required
              />
              <input
                type="submit"
                value="Send"
                className="h-12 px-5 bg-primary text-primary-foreground font-medium transition-colors hover:bg-primary/90 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring"
                style={{ borderRadius: "30px", overflow: "hidden" }}
              />
            </form>
            <div className="grid gap-2">
              <div className="flex items-center gap-2">
                <PhoneIcon className="h-5 w-5 text-muted-foreground" />
                <p className="text-sm text-muted-foreground">
                  +94 (075) 474-5359
                </p>
              </div>
              <div className="flex items-center gap-2">
                <MailIcon className="h-5 w-5 text-muted-foreground" />
                <p className="text-sm text-muted-foreground">
                  info@auradigitallabs.com
                </p>
              </div>
              <div className="flex items-center gap-2">
                <MapPinIcon className="h-5 w-5 text-muted-foreground" />
                <p className="text-sm text-muted-foreground">
                  1st Lane, Moratuwa, Sri Lanka
                </p>
              </div>
            </div>
          </div>
        </div>
      </section>
        );
}