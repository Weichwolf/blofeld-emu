// SPDX-License-Identifier: GPL-3.0-only
// Offline instruction inspection for a fresh, disposable Ghidra project.
// @category Blofeld
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Instruction;
import java.io.PrintWriter;

public class DumpHc12 extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        Address current = currentProgram.getMinAddress().add(Long.parseLong(args[1]));
        int count = Integer.parseInt(args[2]);
        try (PrintWriter out = new PrintWriter(args[0])) {
            out.println("Language: " + currentProgram.getLanguageID());
            for (int i = 0; i < count && current.compareTo(currentProgram.getMaxAddress()) <= 0; i++) {
                Instruction instruction = getInstructionAt(current);
                if (instruction == null) {
                    disassemble(current);
                    instruction = getInstructionAt(current);
                }
                if (instruction == null) {
                    out.println(current + "  undecoded " + Integer.toHexString(getByte(current) & 255));
                    current = current.add(1);
                } else {
                    out.println(current + "  " + instruction);
                    current = current.add(instruction.getLength());
                }
            }
        }
    }
}
