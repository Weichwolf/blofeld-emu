Documentation map

Current scope and evidence
  objective.txt               Complete acceptance objective; not a completion claim
  coverage.txt                Current requirement-by-requirement state and gaps
  function-acceptance.tsv     Every documented Desktop/Keyboard function
  function-acceptance.txt     Rules for qualifying those function rows

Running and viewing
  runtime.txt                 Available runtime interface and current limits
  display-export.txt          Display file format, publication and GTK viewer
  host-midi-session.txt       Host MIDI input and native-output session contract
  midi-output.txt             Native serial output capture and scoped evidence
  host-audio.txt              Session audio negotiation and transport
  own-dsp-integration.txt     Current independent-core composition

References
  dependencies.json          Pinned source dependencies
  downloads.json             Official local-input download identities
  firmware-nand.txt          Complete cold NAND payloads and explicit state policies
  hardware/sources.json      Primary hardware documentation
  *-reference.json           Scoped test inputs/results, with source identities

Implementation belongs in ../src/ and tests in ../test/. The viewer implementation
is ../src/gui/; ../tools/blofeld-display is its thin launcher. Ignored local
references and experiment artifacts stay in local/ and are not product programs
or published manufacturer assets. Earlier bring-up narratives are available in
Git history rather than repeated as current project status.

Low-level documents describe the named component contracts. Their fixture
results do not establish the complete instrument. Use coverage.txt and the
function record for present acceptance status.
