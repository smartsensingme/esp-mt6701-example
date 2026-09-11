# Repository guidance for coding agents

## Comprehensive component documentation profile (opt-in)

This repository has a detailed documentation style intended for components
that are being finalized for teaching, reuse, or release. **Do not apply this
profile automatically.** Apply it only when the user explicitly requests
comprehensive/final component documentation, asks for the established
documentation pattern, or explicitly invokes this profile.

For ordinary implementation, diagnosis, bug fixes, and refactoring, update only
the documentation directly affected by the behavior change. Do not expand
unrelated comments or READMEs merely to match this profile. This opt-in rule
keeps routine changes focused and avoids unnecessary review effort and token
consumption.

When the profile is explicitly requested:

1. Read the component's source, public headers, README, Kconfig, build files,
   examples, and directly coupled application integration before documenting.
   Describe the code that actually exists; do not infer an obsolete design.
2. In public headers, document every public function, type, enum, field, macro,
   and callback. State purpose, units, valid ranges, ownership and lifetime,
   preconditions, state changes, concurrency/real-time constraints, parameters,
   return values, and relevant failure modes.
3. For every function, say whether it is called externally, internally, or
   both. When internal functions call it, name those callers. When understanding
   the implementation benefits from it, identify important functions it calls.
4. In source files, document every public and private function. Divide each
   implementation into functional blocks with comments explaining how and why
   the block works. Add local comments where a non-obvious invariant,
   conversion, synchronization rule, saturation rule, or protocol decision
   matters. Avoid comments that merely restate C syntax.
5. Make the component README sufficient for a new user to integrate it. Cover
   purpose and boundaries, architecture, lifecycle/state machine, configuration
   and Kconfig fields, complete usage examples, API behavior, data formats,
   units, memory ownership, concurrency, deterministic-loop considerations,
   error handling, and troubleshooting as applicable.
6. For wire protocols or stored formats, document framing, byte order, layout,
   scaling/reconstruction, reserved values, integrity checks, retry behavior,
   and compatibility/version rules.
7. Improve Kconfig help so each option explains behavior, resource cost, and
   important interactions instead of only repeating its prompt.
8. If APIs were intentionally removed, summarize that fact in a compatibility
   or removal log; do not preserve obsolete functions solely for documentation.
9. Preserve the language and file set already used by the component. Do not add
   translated or duplicate documentation unless the user requests it or the
   component already maintains those documents in parallel.
10. Finish with formatting, relevant builds/tests, and `git diff --check`.
    Review the resulting documentation against the current implementation and
    correct statements that are broader than the code guarantees.

User instructions for a particular task override this profile. The profile
governs documentation depth, not permission to change behavior or public APIs.

## Independent ESP-IDF component extraction profile (opt-in)

Apply this profile only when the user explicitly asks to transform a component
of this project into an external or independent component. Do not externalize
components automatically during ordinary refactoring.

When this profile is requested:

1. Audit the component boundary before extracting it. Identify its public API,
   source files, Kconfig options, tests, examples, documentation, ESP-IDF
   dependencies, and dependencies on other project components. Move
   application-specific behavior out of the component when that can be done
   without weakening its generality.
2. Make the extracted repository usable at its root as an ESP-IDF component.
   It should contain its own `CMakeLists.txt`, public headers, source files,
   Kconfig when applicable, README, license information, and directly relevant
   tests or examples. Do not require the consumer to reproduce files from this
   application's `main/` directory.
3. Preserve both kinds of dependency declaration:
   - use `REQUIRES` or `PRIV_REQUIRES` in the component's `CMakeLists.txt` to
     describe the ESP-IDF compile/link relationship;
   - create `idf_component.yml` in the root of the independent component to
     tell the ESP-IDF Component Manager how to obtain every external component
     dependency.
4. For a Git dependency in the component's `idf_component.yml`, use a portable
   public URL such as `https://github.com/OWNER/REPOSITORY.git`. Pin a tested
   tag or full commit hash instead of relying silently on a moving branch.
   Declare the supported ESP-IDF version explicitly. Never put a developer's
   SSH host alias, filesystem path, credential, or token in a tracked manifest.
5. Document the consumer-side installation using the application's
   `main/idf_component.yml`. The application should list only the component it
   directly consumes; the installed component's root `idf_component.yml` must
   resolve its transitive dependencies automatically. Make this distinction
   explicit: `main/idf_component.yml` belongs to the consuming application,
   while the component's `idf_component.yml` belongs to the reusable
   component.
6. Keep a manual/offline installation path documented when practical. If the
   Component Manager would otherwise select `managed_components/`, explain how
   to disable it with `IDF_COMPONENT_MANAGER=0` or how to use an explicit local
   override. Do not claim that a sibling local component automatically wins
   unless that behavior was verified with the supported ESP-IDF version.
7. Preserve useful Git history when extracting the component. Create and
   verify the independent component commit before changing the parent project.
   Then replace the in-tree directory with a Git submodule whose tracked URL in
   `.gitmodules` is the standard public HTTPS GitHub URL.
8. Keep authentication configuration local. A developer-specific SSH alias may
   be configured as `remote.origin.pushurl` to select the correct publishing
   key, while `remote.origin.url` and every tracked file remain portable HTTPS
   URLs. Never commit the alias. Do not push unless the user explicitly asks;
   when the user intends to run credentialed commands, provide the exact push
   commands and the required order.
9. Publish dependencies before dependents, and publish the independent
   component before the parent commit that points its submodule at the new
   revision. This prevents the parent repository or a transitive manifest from
   temporarily referencing commits that cannot be fetched.
10. Verify the result with `git diff --check`, a component-relevant test, and an
    ESP-IDF configure/build. For components with transitive dependencies, also
    verify that the Component Manager resolves them from a consumer that lists
    only the top-level component. Confirm that generated
    `managed_components/` content is not committed and decide deliberately
    whether the consuming application should commit `dependencies.lock`.
11. Report the independent-component commit and the parent integration commit
    separately. Also report unpushed commits, required publication order, and
    any unrelated working-tree changes that were intentionally left untouched.
