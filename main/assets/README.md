# Embedded build assets

`nuke_universal.uf2` is downloaded once during `idf.py reconfigure/build` from the pinned Raspberry Pi pico-sdk-prebuilts v2.3.0-0 release and verified against SHA-256 `f1d80fcc37161df8cbdf16a58c111208f06234976a470c17699270d212e91754`.

It is then embedded into the ESP32 application image. Runtime Flash Nuke therefore requires no Internet connection.
