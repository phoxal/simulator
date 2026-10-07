//! Generate runtime dispatch from the unmodified upstream ABI declarations.
//! All inputs are package-local; no native library, headers or compiler are needed.
use quote::quote;
use syn::{ForeignItem, Item, Pat};

fn main() {
    // The retained upstream ABI has optional viewer annotations. This package
    // never enables that upstream feature, but Rust's cfg validator must know it.
    println!("cargo:rustc-check-cfg=cfg(feature, values(\"viewer\"))");
    let input = "src/native_binding/upstream_ffi.rs";
    println!("cargo:rerun-if-changed={input}");
    let source = std::fs::read_to_string(input).expect("packaged ABI declarations");
    let file = syn::parse_file(&source).expect("upstream Rust ABI");
    let mut types = Vec::new();
    let mut fields = Vec::new();
    let mut loads = Vec::new();
    let mut forwards = Vec::new();
    for item in file.items {
        if let Item::ForeignMod(block) = item {
            for item in block.items {
                if let ForeignItem::Fn(function) = item {
                    let variadic = function.sig.variadic.as_ref().map(|_| quote!(, ...));
                    let name = function.sig.ident;
                    let output = function.sig.output;
                    let inputs = function.sig.inputs;
                    let mut args = Vec::new();
                    let mut argument_types = Vec::new();
                    for argument in &inputs {
                        let syn::FnArg::Typed(argument) = argument else {
                            panic!("C receiver");
                        };
                        let Pat::Ident(identifier) = &*argument.pat else {
                            panic!("C argument name");
                        };
                        args.push(identifier.ident.clone());
                        argument_types.push(argument.ty.clone());
                    }
                    fields.push(quote!(pub #name: unsafe extern "C" fn(#(#argument_types),* #variadic) #output));
                    loads.push(quote!(#name: unsafe { *library.get(concat!(stringify!(#name), "\0").as_bytes()).map_err(|error| format!("MuJoCo symbol {}: {error}", stringify!(#name)))? }));
                    forwards.push(quote!(pub unsafe extern "C" fn #name(#inputs) #output { unsafe { (super::api().#name)(#(#args),*) } }));
                }
            }
        } else {
            types.push(item);
        }
    }
    let generated = quote! {
        #(#types)*
        pub(super) struct Api {
            #(#fields,)*
            // The library stays loaded for the complete process lifetime, including object drops.
            _library: libloading::Library,
        }
        impl Api {
            pub(super) unsafe fn load(library: libloading::Library) -> Result<Self, String> {
                Ok(Self { #(#loads,)* _library: library })
            }
        }
        #(#forwards)*
    };
    let file = syn::parse2(generated).expect("generated runtime dispatch");
    let output = std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("Cargo OUT_DIR"));
    std::fs::write(
        output.join("mujoco_dispatch.rs"),
        prettyplease::unparse(&file),
    )
    .expect("write dispatch");
}
