//See LICENSE for license details.
package firesim.midasexamples

import java.io.File
import scala.util.matching.Regex
import scala.io.Source
import org.scalatest.Suites

abstract class TutorialSuite(
    val targetName: String, // See GeneratorUtils
    targetConfigs: String = "NoConfig",
    platformConfigs: String = "HostDebugFeatures_DefaultF1Config",
    tracelen: Int = 8,
    simulationArgs: Seq[String] = Seq()
  ) extends firesim.TestSuiteCommon {

  val backendSimulator = "verilator"

  val targetTuple = s"$targetName-$targetConfigs-$platformConfigs"
  val commonMakeArgs = Seq(s"TARGET_PROJECT=midasexamples",
                           s"DESIGN=$targetName",
                           s"TARGET_CONFIG=${targetConfigs}",
                           s"PLATFORM_CONFIG=${platformConfigs}")

  def run(backend: String,
          debug: Boolean = false,
          logFile: Option[File] = None,
          waveform: Option[File] = None,
          args: Seq[String] = Nil) = {
    val makeArgs = Seq(
      s"run-$backend%s".format(if (debug) "-debug" else ""),
      "LOGFILE=%s".format(logFile map toStr getOrElse ""),
      "WAVEFORM=%s".format(waveform map toStr getOrElse ""),
      "ARGS=%s".format(args mkString " "))
    if (isCmdAvailable(backend)) {
      make(makeArgs:_*)
    } else 0
  }


  def runTest(b: String, debug: Boolean = false) {
    compileMlSimulator(b, debug)
    val testEnv = s"${b} MIDAS-level simulation" + { if (debug) " with waves enabled" else "" }
    if (isCmdAvailable(b)) {
      it should s"pass in ${testEnv}" in {
        assert(run(b, debug, args = simulationArgs) == 0)
      }
    } else {
      ignore should s"pass in ${testEnv}" in { }
    }
  }

  /**
    * Extracts all lines in a file that begin with a specific prefix, removing
    * extra whitespace between the prefix and the remainder of the line
    */
  def extractLines(filename: File, prefix: String, linesToDrop: Int = 0): Seq[String] = {
    // Drop the first line from all files as it is either a header in the synthesized file,
    // or some unrelated output from verlator
    val lines = Source.fromFile(filename).getLines.toList.drop(1)
    lines.filter(_.startsWith(prefix))
         .dropRight(linesToDrop)
         .map(_.stripPrefix(prefix).replaceAll(" +", " "))
  }

  def diffLines(expectedLines: Seq[String], actualLines: Seq[String]): Unit = {
    assert(actualLines.size == expectedLines.size && actualLines.nonEmpty,
      s"\nActual output had length ${actualLines.size}. Expected ${expectedLines.size}")
    for ((vPrint, sPrint) <- expectedLines.zip(actualLines)) {
      assert(sPrint == vPrint)
    }
  }

  // Checks that a bridge generated log in ${genDir}/${synthLog} matches output
  // generated directly by the RTL simulator (usually with printfs)
  def diffSynthesizedLog(synthLog: String,
                         stdoutPrefix: String = "SYNTHESIZED_PRINT ",
                         synthPrefix: String  = "SYNTHESIZED_PRINT ",
                         synthLinesToDrop: Int = 0) {
    behavior of s"${synthLog}"
    it should "match the prints generated by the verilated design" in {
      val verilatedLogFile = new File(outDir,  s"/${targetName}.${backendSimulator}.out")
      val synthLogFile = new File(genDir, s"/${synthLog}")
      val verilatedOutput = extractLines(verilatedLogFile, stdoutPrefix).sorted
      val synthPrintOutput = extractLines(synthLogFile, synthPrefix, synthLinesToDrop).sorted
      diffLines(verilatedOutput, synthPrintOutput)
    }
  }

  def expectedFMR(expectedValue: Double, error: Double = 0.0) {
    it should s"run with an FMR between ${expectedValue - error} and ${expectedValue + error}" in {
      val verilatedLogFile = new File(outDir,  s"/${targetName}.${backendSimulator}.out")
      val lines = Source.fromFile(verilatedLogFile).getLines.toList.reverse
      val fmrRegex = raw"^FMR: (\d*\.\d*)".r
      val fmr = lines.collectFirst {
        case fmrRegex(value) => value.toDouble
      }
      assert(fmr.nonEmpty, "FMR value not found.")
      assert(fmr.get >= expectedValue - error)
      assert(fmr.get <= expectedValue + error)
    }
  }

  mkdirs()
  behavior of s"$targetName"
  elaborateAndCompile()
  runTest(backendSimulator)
}

//class PointerChaserF1Test extends TutorialSuite(
//  "PointerChaser", "PointerChaserConfig", simulationArgs = Seq("`cat runtime.conf`"))
class GCDF1Test extends TutorialSuite("GCD")
// Hijack Parity to test all of the Midas-level backends
class ParityF1Test extends TutorialSuite("Parity") {
  runTest("verilator", true)
  runTest("vcs", true)
}
class ShiftRegisterF1Test extends TutorialSuite("ShiftRegister")
class ResetShiftRegisterF1Test extends TutorialSuite("ResetShiftRegister")
class EnableShiftRegisterF1Test extends TutorialSuite("EnableShiftRegister")
class StackF1Test extends TutorialSuite("Stack")
class RiscF1Test extends TutorialSuite("Risc")
class RiscSRAMF1Test extends TutorialSuite("RiscSRAM")
class AssertModuleF1Test extends TutorialSuite("AssertModule")
class AutoCounterModuleF1Test extends TutorialSuite("AutoCounterModule",
    simulationArgs = Seq("+autocounter-readrate=1000", "+autocounter-filename=AUTOCOUNTERFILE")) {
  diffSynthesizedLog("AUTOCOUNTERFILE0", stdoutPrefix = "AUTOCOUNTER_PRINT ", synthPrefix = "")
}
class AutoCounterCoverModuleF1Test extends TutorialSuite("AutoCounterCoverModule",
    simulationArgs = Seq("+autocounter-readrate=1000", "+autocounter-filename=AUTOCOUNTERFILE")) {
  diffSynthesizedLog("AUTOCOUNTERFILE0", stdoutPrefix = "AUTOCOUNTER_PRINT ", synthPrefix = "")

}
class AutoCounterPrintfF1Test extends TutorialSuite("AutoCounterPrintfModule",
    simulationArgs = Seq("+print-file=synthprinttest.out"),
    platformConfigs = "AutoCounterPrintf_HostDebugFeatures_DefaultF1Config") {
  diffSynthesizedLog("synthprinttest.out0", stdoutPrefix = "SYNTHESIZED_PRINT CYCLE", synthPrefix = "CYCLE")
}
class PrintfModuleF1Test extends TutorialSuite("PrintfModule",
  simulationArgs = Seq("+print-no-cycle-prefix", "+print-file=synthprinttest.out")) {
  diffSynthesizedLog("synthprinttest.out0")
}
class NarrowPrintfModuleF1Test extends TutorialSuite("NarrowPrintfModule",
  simulationArgs = Seq("+print-no-cycle-prefix", "+print-file=synthprinttest.out")) {
  diffSynthesizedLog("synthprinttest.out0")
}

class PrintfCycleBoundsTestBase(startCycle: Int, endCycle: Int) extends TutorialSuite(
  "PrintfModule",
   simulationArgs = Seq(
      "+print-file=synthprinttest.out",
      s"+print-start=${startCycle}",
      s"+print-end=${endCycle}"
    )) {

  // Check that we are extracting from the desired ROI by checking that the
  // bridge-inserted cycle prefix matches the target-side cycle prefix
  it should "have synthesized printfs in the desired cycles" in {
    val synthLogFile = new File(genDir, s"/synthprinttest.out0")
    val synthPrintOutput = extractLines(synthLogFile, prefix = "")
    val length = synthPrintOutput.size
    val printfsPerCycle = 4
    assert(length  == printfsPerCycle * (endCycle - startCycle + 1))
    for ((line, idx) <- synthPrintOutput.zipWithIndex) {
      val currentCycle = idx / printfsPerCycle + startCycle
      val printRegex = raw"^CYCLE:\s*(\d*) SYNTHESIZED_PRINT CYCLE:\s*(\d*).*".r
      line match {
        case printRegex(cycleA, cycleB) =>
          assert(cycleA.toInt == currentCycle)
          assert(cycleB.toInt == currentCycle)
      }
    }
  }
}

class PrintfCycleBoundsF1Test extends PrintfCycleBoundsTestBase(startCycle = 172, endCycle = 9377)

class WireInterconnectF1Test extends TutorialSuite("WireInterconnect")
class TrivialMulticlockF1Test extends TutorialSuite("TrivialMulticlock") {
  runTest("verilator", true)
  runTest("vcs", true)
}

class TriggerWiringModuleF1Test extends TutorialSuite("TriggerWiringModule")

class MulticlockAssertF1Test extends TutorialSuite("MulticlockAssertModule")

class AssertTortureTest extends TutorialSuite("AssertTorture") with AssertTortureConstants {
  def checkClockDomainAssertionOrder(clockIdx: Int): Unit = {
    it should s"capture asserts in the same order as the reference printfs in clock domain $clockIdx" in {
      val verilatedLogFile = new File(outDir,  s"/${targetName}.verilator.out")
      // Diff parts of the simulation's stdout against itself, as the synthesized
      // assertion messages are dumped to the same file as printfs in the RTL
      val expected = extractLines(verilatedLogFile, prefix = s"${printfPrefix}${clockPrefix(clockIdx)}")
      val actual  = extractLines(verilatedLogFile, prefix = s"Assertion failed: ${clockPrefix(clockIdx)}")
      diffLines(expected, actual)
    }
  }
  // TODO: Create a target-parameters instance we can inspect here
  Seq.tabulate(4)(i => checkClockDomainAssertionOrder(i))
}

class MulticlockPrintF1Test extends TutorialSuite("MulticlockPrintfModule",
  simulationArgs = Seq("+print-file=synthprinttest.out",
                       "+print-no-cycle-prefix")) {
  diffSynthesizedLog("synthprinttest.out0")
  diffSynthesizedLog("synthprinttest.out1",
    stdoutPrefix = "SYNTHESIZED_PRINT_HALFRATE ",
    synthPrefix = "SYNTHESIZED_PRINT_HALFRATE ",
    synthLinesToDrop = 4) // Corresponds to a single cycle of extra output
}

class MulticlockAutoCounterF1Test extends TutorialSuite("MulticlockAutoCounterModule",
    simulationArgs = Seq("+autocounter-readrate=1000", "+autocounter-filename=AUTOCOUNTERFILE")) {
  diffSynthesizedLog("AUTOCOUNTERFILE0", "AUTOCOUNTER_PRINT ", "")
  diffSynthesizedLog("AUTOCOUNTERFILE1", "AUTOCOUNTER_PRINT_THIRDRATE ", "")
}
// Basic test for deduplicated extracted models
class TwoAddersF1Test extends TutorialSuite("TwoAdders")

class RegfileF1Test extends TutorialSuite("Regfile")

class MultiRegfileF1Test extends TutorialSuite("MultiRegfile")

class MultiSRAMF1Test extends TutorialSuite("MultiSRAM")

class NestedModelsF1Test extends TutorialSuite("NestedModels")

class MultiRegF1Test extends TutorialSuite("MultiReg")

class PassthroughModelTest extends TutorialSuite("PassthroughModel") {
  expectedFMR(2.0)
}

class PassthroughModelNestedTest extends TutorialSuite("PassthroughModelNested") {
  expectedFMR(2.0)
}

class PassthroughModelBridgeSourceTest extends TutorialSuite("PassthroughModelBridgeSource") {
  expectedFMR(1.0)
}

// Suite Collections
class ChiselExampleDesigns extends Suites(
  new GCDF1Test,
  new ParityF1Test,
  new ResetShiftRegisterF1Test,
  new EnableShiftRegisterF1Test,
  new StackF1Test,
  new RiscF1Test,
  new RiscSRAMF1Test
)

class PrintfSynthesisCITests extends Suites(
  new PrintfModuleF1Test,
  new NarrowPrintfModuleF1Test,
  new MulticlockPrintF1Test,
  new PrintfCycleBoundsF1Test
)

class AssertionSynthesisCITests extends Suites(
  new AssertModuleF1Test,
  new MulticlockAssertF1Test,
  new AssertTortureTest
)

class AutoCounterCITests extends Suites(
  new AutoCounterModuleF1Test,
  new AutoCounterCoverModuleF1Test,
  new AutoCounterPrintfF1Test,
  new MulticlockAutoCounterF1Test
)

class GoldenGateMiscCITests extends Suites(
  new TwoAddersF1Test,
  new TriggerWiringModuleF1Test,
  new WireInterconnectF1Test,
  new TrivialMulticlockF1Test,
  new RegfileF1Test,
  new MultiRegfileF1Test,
  new MultiSRAMF1Test,
  new NestedModelsF1Test,
  new MultiRegF1Test
)

// Each group runs on a single worker instance
class CIGroupA extends Suites(
  new ChiselExampleDesigns,
  new PrintfSynthesisCITests,
  new firesim.fasedtests.CIGroupA,
)

class CIGroupB extends Suites(
  new AssertionSynthesisCITests,
  new GoldenGateMiscCITests,
  new firesim.fasedtests.CIGroupB,
  new firesim.AllMidasUnitTests,
  new firesim.FailingUnitTests
)
