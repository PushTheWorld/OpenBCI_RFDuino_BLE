# OpenBCI Cyton BLE RFDuino Firmware

<p align="center">
  <img alt="banner" src="/images/OBCI_32bit_top.jpg/" width="600">
</p>
<p align="center" href="">
  Drive BLE communications over the RFDuino on the OpenBCI Cyton
</p>

## Welcome!

First and foremost, Welcome! :tada: Willkommen! :confetti_ball: Bienvenue! :balloon::balloon::balloon:

Thank you for visiting the OpenBCI Cyton BLE RFDuino Firmware repository.

This document (the README file) is a hub to give you some information about the project. Jump straight to one of the sections below, or just scroll down to find out more.

* [What are we doing? (And why?)](#what-are-we-doing)
* [Who are we?](#who-are-we)
* [What do we need?](#what-do-we-need)
* [How can you get involved?](#get-involved)
* [Get in touch](#contact-us)
* [Find out more](#find-out-more)
* [Understand the jargon](#glossary)

## What are we doing?

### The problem

* People have to use a dongle to get data from the Cyton
* People can't send data from the Cyton to the web browser :sad_face:
* There is a BLE switch on the Cyton that is doing nothing!

So, these problems add up to limit the amount of devices the cyton can stream it's high quality data to, and that's sad.

### The solution

The OpenBCI Cyton BLE RFDuino Firmware SDK will:

* Advertise a device id as a ble device
* Take data from the Pic32 and send it over BLE
* Prove that BLE is a stable option for the Cyton platform

Using BLE allows for every modern day computer to get data from the Cyton, so let's get this firmware stable!

## Who are we?

The author of the OpenBCI Cyton BLE RFDuino Firmware is [AJ Keller][link_aj_keller] and he was sponsored by [NEBA Health, LLC][link_neba].

We are the OpenBCI community at large though, sourcing our work from dozens of different minds for years. We love brains, we love computers, and we know a bright future is upon us then when we can stream Cyton data into our web browsers and phones.

## What do we need?

**You**! In whatever way you can help.

We need expertise in programming, user experience, software sustainability, documentation and technical writing and project management.

We'd love your feedback along the way.

Our primary goal is to drive BLE communications over the RFDuino on the OpenBCI Cyton and we're excited to support the professional development of any and all of our contributors. If you're looking to learn to code, try out working collaboratively, or translate you skills to the digital domain, we're here to help.

## Get involved

If you think you can help in any of the areas listed above (and we bet you can) or in any of the many areas that we haven't yet thought of (and here we're *sure* you can) then please check out our [contributors' guidelines](CONTRIBUTING.md) and our [roadmap](ROADMAP.md).

Please note that it's very important to us that we maintain a positive and supportive environment for everyone who wants to participate. When you join us we ask that you follow our [code of conduct](CODE_OF_CONDUCT.md) in all interactions both on and offline.

## Contact us

If you want to report a problem or suggest an enhancement we'd love for you to [open an issue](../../issues) at this github repository because then we can get right on it. But you can also contact [AJ][link_aj_keller] by email (pushtheworldllc AT gmail DOT com) or on [twitter](https://twitter.com/aj-ptw).

You can also hang out, ask questions and share stories in the [OpenBCI NodeJS room](https://gitter.im/OpenBCI/OpenBCI_NodeJS?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge) on Gitter.

## Find out more

You might be interested in:

* Check out [NEBA Health, LLC][link_neba]
* An example: [RadioDevice32bit.ino][link_cyton_ble_example]

And of course, you'll want to know our:

* [Contributors' guidelines](CONTRIBUTING.md)
* [Roadmap](ROADMAP.md)

## Thank you

Thank you so much (Danke schön! Merci beaucoup!) for visiting the project and we do hope that you'll join us on this amazing journey to make programming with OpenBCI fun and easy.

# Documentation

### Table of Contents:
---

1. [Installation](#install)
2. [TL;DR](#tldr)
3. [WiFi](#wifi-docs)
  1. [General Overview](#general-overview)
  2. [Classes](#classes)
4. [Developing](#developing)
5. [Testing](#developing-testing)
6. [Contribute](#contribute)
7. [License](#license)

## <a name="developing"></a> Developing:
### <a name="developing-running"></a> Running:

```
npm install
```

### <a name="developing-testing"></a> Testing:

```
npm test
```

## <a name="contribute"></a> Contribute:

1. Fork it!
2. Branch off of `development`: `git checkout development`
2. Create your feature branch: `git checkout -b my-new-feature`
3. Make changes
4. If adding a feature, please add test coverage.
5. Ensure tests all pass. (`npm test`)
6. Commit your changes: `git commit -m 'Add some feature'`
7. Push to the branch: `git push origin my-new-feature`
8. Submit a pull request. Make sure it is based off of the `development` branch when submitting! :D

## <a name="license"></a> License:

MIT

[link_aj_keller]: https://github.com/aj-ptw
[link_shop_wifi_shield]: https://shop.openbci.com/collections/frontpage/products/wifi-shield?variant=44534009550
[link_shop_ganglion]: https://shop.openbci.com/collections/frontpage/products/pre-order-ganglion-board
[link_shop_cyton]: https://shop.openbci.com/collections/frontpage/products/cyton-biosensing-board-8-channel
[link_shop_cyton_daisy]: https://shop.openbci.com/collections/frontpage/products/cyton-daisy-biosensing-boards-16-channel
[link_ptw]: https://www.pushtheworldllc.com
[link_neba]: https://nebahealth.com
[link_openbci]: http://www.openbci.com
[link_mozwow]: http://mozillascience.github.io/working-open-workshop/index.html
[link_cyton_ble_example]: examples/RadioDevice32bit/RadioDevice32bit.ino
[link_openleaderscohort]: https://medium.com/@MozOpenLeaders
[link_mozsci]: https://science.mozilla.org
