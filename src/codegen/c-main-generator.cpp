#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdarg.h>
#include <filesystem>
#include <algorithm>
#include <regex>
#include "helpers/formatter.h"
#include "mon-generator.h"
#include "config-generator.h"

#include "c-main-generator.h"

const char* ext_sig_func_name = "__ext_sig__";

const char* extend_func_body =
  "// This function performs extension of sign for the signals\n"
  "// which have non-aligned to power of 2 bit's width.\n"
  "// The types 'bitext_t' and 'ubitext_t' define maximal bit width which\n"
  "// can be correctly handled. You need to select type which can contain\n"
  "// n+1 bits where n is the largest signed signal width. For example if\n"
  "// the most wide signed signal has a width of 31 bits you need to set\n"
  "// bitext_t as int32_t and ubitext_t as uint32_t\n"
  "// Defined these typedefs in @dbccodeconf.h or locally in 'dbcdrvname'-config.h\n"
  "static bitext_t %s(ubitext_t val, uint8_t bits)\n"
  "{\n"
  "  ubitext_t const m = 1u << (bits - 1);\n"
  "  return (val ^ m) - m;\n"
  "}\n\n";

CiMainGenerator::CiMainGenerator()
{
  sigprt = std::make_unique<CSigPrinter>();
  fwriter = std::make_unique<FileWriter>();
}

void CiMainGenerator::Generate(DbcMessageList_t& dlist, const AppSettings_t& fsd)
{
  // Load income messages to sig printer
  sigprt->LoadMessages(dlist.msgs);

  // save pointer to output file descriptor struct to
  // enable using this information inside class member functions
  fdesc = &fsd;

  std::sort(sigprt->sigs_expr.begin(), sigprt->sigs_expr.end(),
    [](const CiExpr_t* a, const CiExpr_t* b) -> bool
  {
    return a->msg.MsgID < b->msg.MsgID;
  });

  // 2 step is to print main head file
  Gen_MainHeader();

  // 3 step is to print main source file
  Gen_MainSource();

  // 4 step is to pring fmon head file
  Gen_FMonHeader();

  if (!fsd.gen.no_fmon)
  {
    // 5 step is to print fmon source file
    Gen_FMonSource();
  }

  if (!fsd.gen.no_config)
  {
    // 6 step is to print template for drv-config.h
    Gen_ConfigHeader();

    // 8 step is to print dbccodeconf.h template
    Gen_DbcCodeConf();
  }

  if (!fsd.gen.no_inc)
  {
    // 7 step is to print canmonitorutil.h template code
    Gen_CanMonUtil();
  }

  // ros functionality (changed by AdriveLivingLab)

  // 1 step decode the messages from can into a struct
  Gen_MainDecoder();
  // 2 step convert from raw to physical values
  Gen_MainConverter();
  // 3 step create ros .msg file definitions
  Gen_MainMsgs();
  // 5 Generate code snippet to initialise the .msg files
  Gen_CMake();
  // 6 Generate code to start all publishers
  Gen_Publishers();
  // 7 Generate code to handle the evaluation of a packet
  Gen_Evaluation();
}

void CiMainGenerator::Gen_MainHeader()
{
  // write comment start text
  if (fdesc->gen.start_info.size() > 0)
  {
    // replace all '\n' on "\n //" for c code comment text
    fwriter->Append("// " + std::regex_replace(fdesc->gen.start_info, std::regex("\n"), "\n// "));
  }

  fwriter->Append("#pragma once");
  fwriter->Append();
  fwriter->Append("#ifdef __cplusplus\nextern \"C\" {\n#endif");
  fwriter->Append();
  fwriter->Append("#include <stdint.h>");
  fwriter->Append();

  fwriter->Append("// DBC file version");
  fwriter->Append("#define %s (%uU)", fdesc->gen.verhigh_def.c_str(), fdesc->gen.hiver);
  fwriter->Append("#define %s (%uU)", fdesc->gen.verlow_def.c_str(), fdesc->gen.lowver);
  fwriter->Append();

  fwriter->Append("// include current dbc-driver compilation config");
  fwriter->Append("#include <%s-config.h>", fdesc->gen.drvname.c_str());
  fwriter->Append();

  fwriter->Append("#ifdef %s", fdesc->gen.usemon_def.c_str());

  fwriter->Append(
    "// This file must define:\n"
    "// base monitor struct\n"
    "#include <canmonitorutil.h>\n"
    "\n"
  );

  fwriter->Append("#endif // %s", fdesc->gen.usemon_def.c_str());
  fwriter->Append(2);

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t& m = sigprt->sigs_expr[num]->msg;

    if (m.CommentText.size() > 0)
    {
      // replace all '\n' on "\n //" for c code comment text
      fwriter->Append("// " + std::regex_replace(m.CommentText, std::regex("\n"), "\n// "));
    }

    fwriter->Append("// def @%s CAN Message (%-4d %#x)", m.Name.c_str(), m.MsgID, m.MsgID);
    fwriter->Append("#define %s_IDE (%uU)", m.Name.c_str(), m.IsExt);
    fwriter->Append("#define %s_DLC (%uU)", m.Name.c_str(), m.DLC);
    fwriter->Append("#define %s_CANID (%#x)", m.Name.c_str(), m.MsgID);

    if (m.Cycle > 0)
    {
      fwriter->Append("#define %s_CYC (%dU)", m.Name.c_str(), m.Cycle);
    }

    size_t max_sig_name_len = 27;

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t& s = m.Signals[signum];

      if (!s.IsSimpleSig)
      {
        fwriter->Append(sigprt->PrintPhysicalToRaw(&s, fdesc->gen.DRVNAME));
      }

      if (s.Name.size() > max_sig_name_len)
      {
        max_sig_name_len = s.Name.size();
      }

      // For each signal in current message print value tables definitions
      if (s.ValDefs.vpairs.size() > 0)
      {
        fwriter->Append("\n// Value tables for @%s signal", s.Name.c_str());
        fwriter->Append();

        for (auto i = 0; i < s.ValDefs.vpairs.size(); i++)
        {
          // The value table definition consists of 'signal name + message name + value definition'
          // This provides reliable way of avoiding issues with same macros names
          std::string defname = StrPrint("%s_%s_%s", s.Name.c_str(), m.Name.c_str(), s.ValDefs.vpairs[i].first.c_str());

          // @ifndef guard for the case when different values of table have
          // the same name (it is valid for DBC file format)
          // For this case only one of same named values will be available as macro
          fwriter->Append("#ifndef %s", defname.c_str());

          fwriter->Append("#define %s_%s_%s (%d)",
            s.Name.c_str(), m.Name.c_str(), s.ValDefs.vpairs[i].first.c_str(),
            s.ValDefs.vpairs[i].second);

          fwriter->Append("#endif");
          fwriter->Append();
        }
      }
    }

    fwriter->Append();
    fwriter->Append("typedef struct");
    fwriter->Append("{");

    // Write section for bitfielded part
    fwriter->Append("#ifdef %s", fdesc->gen.usebits_def.c_str());
    fwriter->Append();

    SignalDescriptor_t rollsig;

    if (m.RollSig != nullptr)
    {
      // rolling counter is detected
      rollsig = (*m.RollSig);
      rollsig.CommentText = "";
      rollsig.Name += "_expt";
    }

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t& sig = m.Signals[signum];
      // Write bit-fielded part
      WriteSigStructField(sig, true, max_sig_name_len);
    }

    if (m.RollSig != nullptr)
    {
      fwriter->Append("#ifdef %s", fdesc->gen.useroll_def.c_str());
      fwriter->Append();
      WriteSigStructField(rollsig, true, max_sig_name_len);
      fwriter->Append("#endif // %s", fdesc->gen.useroll_def.c_str());
      fwriter->Append();
    }

    // Write clean part
    fwriter->Append("#else");
    fwriter->Append();

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t& sig = m.Signals[signum];
      // Write clean signals
      WriteSigStructField(sig, false, max_sig_name_len);
    }

    if (m.RollSig != nullptr)
    {
      fwriter->Append("#ifdef %s", fdesc->gen.useroll_def.c_str());
      fwriter->Append();
      WriteSigStructField(rollsig, false, max_sig_name_len);
      fwriter->Append("#endif // %s", fdesc->gen.useroll_def.c_str());
      fwriter->Append();
    }

    fwriter->Append("#endif // %s", fdesc->gen.usebits_def.c_str());
    fwriter->Append();

    // start mon1 section
    fwriter->Append("#ifdef %s", fdesc->gen.usemon_def.c_str());
    fwriter->Append();
    fwriter->Append("  FrameMonitor_t mon1;");
    fwriter->Append();
    fwriter->Append("#endif // %s", fdesc->gen.usemon_def.c_str());
    fwriter->Append();
    fwriter->Append("} %s_t;", m.Name.c_str());
    fwriter->Append();
  }

  fwriter->Append("// Function signatures");
  fwriter->Append();

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t& m = sigprt->sigs_expr[num]->msg;

    fwriter->Append("uint32_t Unpack_%s_%s(%s_t* _m, const uint8_t* _d, uint8_t dlc_);",
      m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());

    fwriter->Append("#ifdef %s", fdesc->gen.usesruct_def.c_str());

    fwriter->Append("uint32_t Pack_%s_%s(%s_t* _m, __CoderDbcCanFrame_t__* cframe);",
      m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());

    fwriter->Append("#else");

    fwriter->Append("uint32_t Pack_%s_%s(%s_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);",
      m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());

    fwriter->Append("#endif // %s", fdesc->gen.usesruct_def.c_str());
    fwriter->Append();
  }

  fwriter->Append("#ifdef __cplusplus\n}\n#endif");

  // save fwrite cached text to file
  fwriter->Flush(fdesc->file.core_h.fpath);
}

void CiMainGenerator::Gen_MainSource()
{
  if (fdesc->gen.start_info.size() > 0)
  {
    // replace all '\n' on "\n //" for c code comment text
    fwriter->Append("// " + std::regex_replace(fdesc->gen.start_info, std::regex("\n"), "\n// "));
  }

  // include main header file
  fwriter->Append("#include \"%s\"", fdesc->file.core_h.fname.c_str());
  fwriter->Append(2);

  fwriter->Append("// DBC file version");
  fwriter->Append("#if (%s != (%uU)) || (%s != (%uU))",
    fdesc->gen.verhigh_def.c_str(), fdesc->gen.hiver, fdesc->gen.verlow_def.c_str(), fdesc->gen.lowver);

  fwriter->Append("#error The %s dbc source files have different versions", fdesc->gen.DRVNAME.c_str());
  fwriter->Append("#endif");
  fwriter->Append();

  // put diagmonitor ifdef selection for including @drv-fmon header
  // with FMon_* signatures to call from unpack function
  fwriter->Append("#ifdef %s", fdesc->gen.usemon_def.c_str());

  fwriter->Append(
    "// Function prototypes to be called each time CAN frame is unpacked\n"
    "// FMon function may detect RC, CRC or DLC violation\n");

  fwriter->Append("#include <%s-fmon.h>", fdesc->gen.drvname.c_str());
  fwriter->Append();

  fwriter->Append("#endif // %s", fdesc->gen.usemon_def.c_str());
  fwriter->Append("");
  fwriter->Append("// This macro guard for the case when you need to enable");
  fwriter->Append("// using diag monitors but there is no necessity in proper");
  fwriter->Append("// SysTick provider. For providing one you need define macro");
  fwriter->Append("// before this line - in dbccodeconf.h");
  fwriter->Append("");
  fwriter->Append("#ifndef GetSystemTick");
  fwriter->Append("#define GetSystemTick() (0u)");
  fwriter->Append("#endif");
  fwriter->Append("");
  fwriter->Append("// This macro guard is for the case when you want to build");
  fwriter->Append("// app with enabled optoin auto CSM, but don't yet have");
  fwriter->Append("// proper getframehash implementation");
  fwriter->Append("");
  fwriter->Append("#ifndef GetFrameHash");
  fwriter->Append("#define GetFrameHash(a,b,c,d,e) (0u)");
  fwriter->Append("#endif");
  fwriter->Append();

  fwriter->Append(extend_func_body, ext_sig_func_name), 1;

  // for each message 3 functions must be defined - 1 unpack function,
  // 2: pack with raw signature
  // 3: pack with canstruct
  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t& m = sigprt->sigs_expr[num]->msg;

    // first function
    fwriter->Append("uint32_t Unpack_%s_%s(%s_t* _m, const uint8_t* _d, uint8_t dlc_)\n{",
      m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());

    // put dirt trick to avoid warning about unusing parameter
    // (dlc) when monitora are disabled. trick is better than
    // selection different signatures because of external API consistency
    fwriter->Append("  (void)dlc_;");

    WriteUnpackBody(sigprt->sigs_expr[num]);

    fwriter->Append("}");
    fwriter->Append();

    // next one is the pack function for using with CANFrame struct
    fwriter->Append("#ifdef %s", fdesc->gen.usesruct_def.c_str());
    fwriter->Append();

    // second function
    fwriter->Append("uint32_t Pack_%s_%s(%s_t* _m, __CoderDbcCanFrame_t__* cframe)",
      m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());

    WritePackStructBody(sigprt->sigs_expr[num]);

    fwriter->Append("#else");
    fwriter->Append();

    // third function
    fwriter->Append("uint32_t Pack_%s_%s(%s_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)",
      m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());

    WritePackArrayBody(sigprt->sigs_expr[num]);

    fwriter->Append("#endif // %s", fdesc->gen.usesruct_def.c_str());
    fwriter->Append();
  }

  fwriter->Flush(fdesc->file.core_c.fpath);
}

void CiMainGenerator::Gen_ConfigHeader()
{
  if (fdesc->gen.start_info.size() > 0)
  {
    // replace all '\n' on "\n //" for c code comment text
    fwriter->Append("// " + std::regex_replace(fdesc->gen.start_info, std::regex("\n"), "\n// "));
  }

  ConfigGenerator confgen;
  confgen.FillHeader((*fwriter), fdesc->gen);

  fwriter->Flush(fdesc->file.confdir + '/' + fdesc->gen.drvname + "-config.h");
}

void CiMainGenerator::Gen_FMonHeader()
{
  MonGenerator mongen;
  mongen.FillHeader((*fwriter), sigprt->sigs_expr, *fdesc);
  fwriter->Flush(fdesc->file.fmon_h.fpath);
}

void CiMainGenerator::Gen_FMonSource()
{
  MonGenerator mongen;
  mongen.FillSource((*fwriter), sigprt->sigs_expr, *fdesc);
  fwriter->Flush(fdesc->file.fmon_c.fpath);
}

void CiMainGenerator::Gen_CanMonUtil()
{
  fwriter->Append("#pragma once");
  fwriter->Append("");
  fwriter->Append("#include <stdint.h>");
  fwriter->Append("");
  fwriter->Append("#ifdef __cplusplus");
  fwriter->Append("extern \"C\" {");
  fwriter->Append("#endif");
  fwriter->Append("");
  fwriter->Append("// declare here all availible checksum algorithms");
  fwriter->Append("typedef enum");
  fwriter->Append("{");
  fwriter->Append("  // XOR8 = 0,");
  fwriter->Append("  // XOR4 = 1,");
  fwriter->Append("  // etc");
  fwriter->Append("");
  fwriter->Append("  // it is up to user to have or to skip final enum value - @CRC_ALG_COUNT");
  fwriter->Append("  CRC_ALG_COUNT");
  fwriter->Append("} DbcCanCrcMethods;");
  fwriter->Append("");
  fwriter->Append("typedef struct");
  fwriter->Append("{");
  fwriter->Append("  // @last_cycle keeps tick-value when last frame was received");
  fwriter->Append("  uint32_t last_cycle;");
  fwriter->Append("");
  fwriter->Append("  // @timeout_cycle keeps maximum timeout for frame, user responsibility");
  fwriter->Append("  // to init this field and use it in missing frame monitoring function");
  fwriter->Append("  uint32_t timeout_cycle;");
  fwriter->Append("");
  fwriter->Append("  // @frame_cnt keeps count of all the received frames");
  fwriter->Append("  uint32_t frame_cnt;");
  fwriter->Append("");
  fwriter->Append("  // setting up @roll_error bit indicates roll counting fail.");
  fwriter->Append("  // Bit is not clearing automatically!");
  fwriter->Append("  uint32_t roll_error : 1;");
  fwriter->Append("");
  fwriter->Append("  // setting up @checksum_error bit indicates checksum checking failure.");
  fwriter->Append("  // Bit is not clearing automatically!");
  fwriter->Append("  uint32_t csm_error : 1;");
  fwriter->Append("");
  fwriter->Append("  // setting up @cycle_error bit indicates that time was overrunned.");
  fwriter->Append("  // Bit is not clearing automatically!");
  fwriter->Append("  uint32_t cycle_error : 1;");
  fwriter->Append("");
  fwriter->Append("  // setting up @dlc_error bit indicates that the actual length of");
  fwriter->Append("  // CAN frame is less then defined by CAN matrix!");
  fwriter->Append("  uint32_t dlc_error : 1;");
  fwriter->Append("");
  fwriter->Append("} FrameMonitor_t;");
  fwriter->Append("");
  fwriter->Append("#ifdef __cplusplus");
  fwriter->Append("}");
  fwriter->Append("#endif");
  fwriter->Append("");

  fwriter->Flush(fdesc->file.incdir + '/' + "canmonitorutil.h");
}

void CiMainGenerator::Gen_DbcCodeConf()
{
  fwriter->Append("#pragma once");
  fwriter->Append("");
  fwriter->Append("#include <stdint.h>");
  fwriter->Append("");
  fwriter->Append("// when USE_SIGFLOAT enabed the sigfloat_t must be defined");
  fwriter->Append("// typedef double sigfloat_t;");
  fwriter->Append("");
  fwriter->Append("// when USE_CANSTRUCT enabled __CoderDbcCanFrame_t__ must be defined");
  fwriter->Append("// #include \"{header_with_can_struct}\"");
  fwriter->Append("// typedef {can_struct} __CoderDbcCanFrame_t__;");
  fwriter->Append("");
  fwriter->Append("// if you need to allocate rx and tx messages structs put the allocation macro here");
  fwriter->Append("// #define __DEF_{your_driver_name}__");
  fwriter->Append("");

  fwriter->Append("// defualt @__ext_sig__ help types definition");
  fwriter->Append("");
  fwriter->Append("typedef uint32_t ubitext_t;");
  fwriter->Append("typedef int32_t bitext_t;");
  fwriter->Append("");
  fwriter->Append("// To provide a way to make missing control correctly you");
  fwriter->Append("// have to define macro @GetSystemTick() which has to");
  fwriter->Append("// return kind of tick counter (e.g. 1 ms ticker)");
  fwriter->Append("");
  fwriter->Append("// #define GetSystemTick() __get__tick__()");
  fwriter->Append("");
  fwriter->Append("// To provide a way to calculate hash (crc) for CAN");
  fwriter->Append("// frame's data field you have to define macro @GetFrameHash");
  fwriter->Append("");
  fwriter->Append("// #define GetFrameHash(a,b,c,d,e) __get_hash__(a,b,c,d,e)");
  fwriter->Append("");

  fwriter->Flush(fdesc->file.confdir + '/' + "dbccodeconf.h");
}

//(added by AdriveLivingLab)
void CiMainGenerator::Gen_MainDecoder() 
{
  // message decoding
  // Example:
  //
  // typedef struct MESSAGE_NAME {
  // u32 SIGNAL_NAME : BIT_LENGTH ; // BIT_LENGTH is signal lenght in frame on ethernet packet and as flexray signal
  // }MESSAGE_NAME;

  fwriter->Append("// typedef struct for message decoding");
  fwriter->Append(2);
  fwriter->Append("#pragma once");
  fwriter->Append("#include <stdint.h>");

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedefs and additional expressions
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;
    size_t max_sig_name_len = 27;
    // get max length of a signal name for alignment of the file
    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t &s = m.Signals[signum];

      if (s.Name.size() > max_sig_name_len)
      {
        max_sig_name_len = s.Name.size();
      }
    }

    /***************************************
     * Decoding part
     ***************************************/
    fwriter->Append();
    fwriter->Append("typedef struct __attribute__((packed, aligned(1))) %s_raw", m.Name.c_str());
    fwriter->Append("{");

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t &sig = m.Signals[signum];
      // Write bit-fielded part
      WriteSimpleSigStructField(sig, true, max_sig_name_len, false, true);
    }
    fwriter->Append("} %s_raw;", m.Name.c_str());
    fwriter->Append();
  }
  // save fwrite cached text to file
  fwriter->Flush(fdesc->file.rosdir + '/' + "decode.h");
}

//(added by AdriveLivingLab)
void CiMainGenerator::Gen_MainConverter()
{
  // typedef for message decoding
  // Example:
  //
  //  void MESSAGE_NAME_APP(pkt)
  //  appStr.SIGAN_NAME                      = pkt->SIGNAL_NAME               * FACTOR + OFFSET             ;
  //  };

  fwriter->Append("// typedef struct for calculation of physical values");
  fwriter->Append(2);
  fwriter->Append("#include <stdint.h>");
  fwriter->Append("#include <stdlib.h>");
  fwriter->Append("#include \"decode.h\"");

  // Includes
  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;
    fwriter->Append("#include <%s/%s.h>", fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());
  }

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;
    size_t max_sig_name_len = 27;

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t &s = m.Signals[signum];

      if (s.Name.size() > max_sig_name_len)
      {
        max_sig_name_len = s.Name.size();
      }
    }

    fwriter->Append();
    fwriter->Append("%s::%s* %s_APP(%s_raw *pkt)", fdesc->gen.DrvName_orig.c_str(), m.Name.c_str(), m.Name.c_str(), m.Name.c_str());
    fwriter->Append("{");
    fwriter->Append("%s::%s *pAppMsg = (%s::%s *) calloc(1,sizeof(%s::%s));", fdesc->gen.DrvName_orig.c_str(), m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());
    fwriter->Append("pAppMsg->header.stamp = ros::Time::now();");
    SignalDescriptor_t rollsig;

    if (m.RollSig != nullptr)
    {
      // rolling counter is detected
      rollsig = (*m.RollSig);
      rollsig.CommentText = "";
      rollsig.Name += "_expt";
    }

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t &sig = m.Signals[signum];
      // Write bit-fielded part
      WriteSimpleSigConvField(sig, false, max_sig_name_len);
    }
    fwriter->Append("return pAppMsg;");
    fwriter->Append("}");
    fwriter->Append();
  }
  // save fwrite cached text to file
  fwriter->Flush(fdesc->file.rosdir + '/' + "convert.h");
}

//(added by AdriveLivingLab)
void CiMainGenerator::Gen_MainMsgs()
{
  // file creation of .msg files
  // Example:
  //  Header header
  //  int SIGNAL_NAME;
  //  };

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;
    size_t max_sig_name_len = 27;

    fwriter->Append("Header header");
    std::string dtype = "";

    for (size_t signum = 0; signum < m.Signals.size(); signum++)
    {
      SignalDescriptor_t &sig = m.Signals[signum];
      // Write bit-fielded part
      WriteRosMsgFile(sig);
    }
    // save fwrite cached text to file
    fwriter->Flush(fdesc->file.rosmsgdir + '/' + m.Name.c_str() + ".msg");
  }
}

//(added by AdriveLivingLab)
void CiMainGenerator::Gen_CMake()
{
  // message including
  // Example:
  //  #include <ROS_PACKAGE_NAME/MESSAGE_NAME.h>
  // save fwrite cached text to file
  fwriter->Append("cmake_minimum_required(VERSION 3.0.2)");
  fwriter->Append("project(%s)", fdesc->gen.DrvName_orig.c_str());

  fwriter->Append("add_compile_options(-std=c++17 -Wno-packed-bitfield-compat)");

  fwriter->Append("find_package(catkin REQUIRED COMPONENTS");
  fwriter->Append("    message_generation");
  fwriter->Append("    roscpp");
  fwriter->Append("    rospy");
  fwriter->Append("    std_msgs");
  fwriter->Append(")");

  fwriter->Append("find_package(Boost REQUIRED COMPONENTS system date_time)");

  fwriter->Append("add_message_files(");
  fwriter->Append("  FILES");

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // get message typedefs and additional expressions
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;
    // write messagename.msg
    fwriter->Append("   %s.msg", m.Name.c_str());
  }

  fwriter->Append(")");
  fwriter->Append(" generate_messages(");
  fwriter->Append("  DEPENDENCIES");
  fwriter->Append("  std_msgs");
  fwriter->Append(" )");

  fwriter->Append("catkin_package(");
  fwriter->Append("  CATKIN_DEPENDS ");
  fwriter->Append("  roscpp");
  fwriter->Append("  std_msgs");
  fwriter->Append("  DEPENDS");
  fwriter->Append("  Boost");
  fwriter->Append("  message_runtime");
  fwriter->Append(")");

  fwriter->Append("set(use_SYSTEM_BOOST TRUE)");

  fwriter->Append("include_directories(");
  fwriter->Append("  include");
  fwriter->Append("  ${catkin_INCLUDE_DIRS}");
  fwriter->Append("  SYSTEM {Boost_INCLUDE_DIRS}");
  fwriter->Append(")");

  fwriter->Append("add_library(${PROJECT_NAME}_core");
  fwriter->Append("  include/%s/decode.h", fdesc->gen.DrvName_orig.c_str());
  fwriter->Append("  include/%s/convert.h",fdesc->gen.DrvName_orig.c_str());
  fwriter->Append("  src/%s.cpp",fdesc->gen.DrvName_orig.c_str());
  fwriter->Append(")");

  fwriter->Append("add_executable(");
  fwriter->Append("  ${PROJECT_NAME} src/%s.cpp",fdesc->gen.DrvName_orig.c_str());
  fwriter->Append(" )");

  fwriter->Append("add_dependencies(${PROJECT_NAME}_core");
  fwriter->Append("${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})");

  fwriter->Append("add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)");

  fwriter->Append("add_dependencies(${PROJECT_NAME}");
  fwriter->Append("  ${catkin_EXPORTED_TARGETS}");
  fwriter->Append(")");

  fwriter->Append("target_link_libraries(${PROJECT_NAME}_core");
  fwriter->Append("  ${catkin_LIBRARIES}");
  fwriter->Append(")");

  fwriter->Append("target_link_libraries(${PROJECT_NAME}");
  fwriter->Append("  ${PROJECT_NAME}_core");
  fwriter->Append("  ${catkin_LIBRARIES}");
  fwriter->Append("  ${Boost_LIBRARIES}");
  fwriter->Append(")");

  fwriter->Append("install(");
  fwriter->Append("  TARGETS ${PROJECT_NAME}");
  fwriter->Append("  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}");
  fwriter->Append(")");

  fwriter->Append("install(TARGETS ${PROJECT_NAME}");
  fwriter->Append("  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}");
  fwriter->Append("  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}");
  fwriter->Append("  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}");
  fwriter->Append(")");

  fwriter->Append("install(");
  fwriter->Append("  DIRECTORY include/${PROJECT_NAME}/");
  fwriter->Append("  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}");
  fwriter->Append("  FILES_MATCHING PATTERN \"*.hpp\"");
  fwriter->Append("  )");

  fwriter->Append("  install(");
  fwriter->Append("  DIRECTORY doc");
  fwriter->Append("  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}");
  fwriter->Append("  )");
  
  fwriter->Flush(fdesc->file.rosdir + '/' + "CMakeLists.txt");
}

//(added by AdriveLivingLab)
void CiMainGenerator::Gen_Publishers()
{
  // message including
  // Example:
  //  ros::Publisher  publisher_ = nh.advertise<ROS_PACKAGE_NAME::MESSAGE_NAME>("FRAME_4_3_4_B", 1);

  fwriter->Append("// Publisher inits");

  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;

    fwriter->Append("ros::Publisher  publish_%s_ = nh.advertise<%s::%s>(\"%s\", 1);", m.Name.c_str(), fdesc->gen.DrvName_orig.c_str(), m.Name.c_str(), m.Name.c_str());
  }
  // save fwrite cached text to file
  fwriter->Flush(fdesc->file.rosdir + '/' + "publishers.txt");
}

void CiMainGenerator::Gen_Evaluation()
{
  for (size_t num = 0; num < sigprt->sigs_expr.size(); num++)
  {
    // write message typedef s and additional expressions
    MessageDescriptor_t &m = sigprt->sigs_expr[num]->msg;

    fwriter->Append("if (pApp->pPkt->dw_id == %d)", m.MsgID);
    fwriter->Append("{");
    // fwriter->Append("%s_raw *rawMsg;",m.Name.c_str());
    fwriter->Append("%s::%s *decodedMsg;", fdesc->gen.DrvName_orig.c_str(), m.Name.c_str());
    fwriter->Append("%s_raw *rawMsg = (%s_raw *)(pApp->pPkt->ab_data);", m.Name.c_str(), m.Name.c_str());
    fwriter->Append("decodedMsg = %s_APP(rawMsg);", m.Name.c_str());
    fwriter->Append("publish_%s_.publish(*decodedMsg);", m.Name.c_str());
    fwriter->Append("free(pApp->pPkt);");
    fwriter->Append("free(decodedMsg);");
    fwriter->Append("}");
  }
  fwriter->Flush(fdesc->file.rosdir + '/' + "evaluation.txt");
}

void CiMainGenerator::WriteSigStructField(const SignalDescriptor_t &sig, bool bits, size_t padwidth)
{
  if (sig.CommentText.size() > 0)
  {
    fwriter->Append("  // " + std::regex_replace(sig.CommentText, std::regex("\n"), "\n  // "));
  }

  if (sig.ValueText.size() > 0)
  {
    fwriter->Append("  // " + std::regex_replace(sig.ValueText, std::regex("\n"), "\n  // "));
  }

  if (sig.Multiplex == MultiplexType::kMulValue)
  {
    fwriter->Append("  // multiplex variable");
  }
  else if (sig.Multiplex == MultiplexType::kMaster)
  {
    fwriter->Append("  // MULTIPLEX master signal");
  }

  std::string dtype = "";

  dtype += "  " + PrintType((int)sig.TypeRo) + " " + sig.Name;

  if (bits && (sig.LengthBit < 8))
  {
    dtype += StrPrint(" : %d", sig.LengthBit);
  }

  dtype += ";";

  std::string pad = " ";

  dtype += pad.insert(0, padwidth + 16 - dtype.size(), ' ');

  fwriter->AppendText(dtype);

  pad = " // ";
  pad += (sig.Signed) ? " [-]" : "    ";

  fwriter->AppendText(pad);

  fwriter->AppendText(" Bits=%2d", sig.LengthBit);

  size_t offset = 0;
  std::string infocmnt{};

  if (sig.IsDoubleSig)
  {
    if (sig.Offset != 0)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 27;
      infocmnt += StrPrint(" Offset= %f", sig.Offset);
    }

    if (sig.Factor != 1)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 24;
      infocmnt += StrPrint(" Factor= %f", sig.Factor);
    }
  }
  else if (sig.IsSimpleSig == false)
  {
    // 2 type of signal
    if (sig.Offset != 0)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 27;
      infocmnt += StrPrint(" Offset= %d", (int)sig.Offset);
    }

    if (sig.Factor != 1)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 24;
      infocmnt += StrPrint(" Factor= %d", (int)sig.Factor);
    }
  }

  if (sig.Unit.size() > 0)
  {
    infocmnt = IndentedString(offset, infocmnt);
    infocmnt += StrPrint(" Unit:'%s'", sig.Unit.c_str());
  }

  fwriter->AppendText(infocmnt);

  fwriter->Append("");
  fwriter->Append();

  if (!sig.IsSimpleSig)
  {
    // this code only required be d-signals (floating point values based)
    // it placed additional signals to struct for conversion
    // to/from physical values. For non-simple and non-double signal
    // there is no necessity to create addition fields
    // @sigfloat_t must be typedefed by user (e.g. double / float)

    // UPD: from this commit, all non-Simple signals has it's
    // own 'shadow' (_phys) copies, the problem with intermediate type (not simpe and
    // not double) is that the x = ***_toS(x) takes place in each Pack_* call
    // the signals which are not changing from Pack_* to Pack_* will change its values (!)
    fwriter->Append("#ifdef %s", fdesc->gen.usesigfloat_def.c_str());

    if (sig.IsDoubleSig)
    {
      fwriter->Append("  sigfloat_t %s;", sig.NameFloat.c_str());
    }
    else
    {
      fwriter->Append("  %s %s;", PrintType((int)sig.TypePhys).c_str(), sig.NameFloat.c_str());
    }

    fwriter->Append("#endif // %s", fdesc->gen.usesigfloat_def.c_str());
    fwriter->Append();
  }
}

//(added by AdriveLivingLab)
void CiMainGenerator::WriteSimpleSigStructField(const SignalDescriptor_t &sig, bool bits, size_t padwidth, bool commenttext, bool rawtype) 
{
  if ((sig.CommentText.size() > 0) && commenttext)
  {
    fwriter->Append("  // " + std::regex_replace(sig.CommentText, std::regex("\n"), "\n  // "));
  }

  if ((sig.ValueText.size() > 0) && commenttext)
  {
    fwriter->Append("  // " + std::regex_replace(sig.ValueText, std::regex("\n"), "\n  // "));
  }

  if (sig.Multiplex == MultiplexType::kMulValue)
  {
    fwriter->Append("  // multiplex variable");
  }
  else if (sig.Multiplex == MultiplexType::kMaster)
  {
    fwriter->Append("  // MULTIPLEX master signal");
  }

  std::string dtype = "";

  if (rawtype || sig.Factor == 1)
  {
    dtype += "  " + PrintType((int)sig.TypeRo) + " " + sig.Name;
  }
  else
  {
    dtype += "  double " + sig.Name;
  }

  if (bits)
  {
    dtype += StrPrint(" : %i", sig.LengthBit);
  }

  dtype += ";";

  std::string pad = " ";

  dtype += pad.insert(0, padwidth + 18 - dtype.size(), ' ');

  fwriter->AppendText(dtype);

  pad = " // ";
  pad += (sig.Signed) ? " [-]" : "    ";

  fwriter->AppendText(pad);

  fwriter->AppendText(" Bits=%2d", sig.LengthBit);

  size_t offset = 0;
  std::string infocmnt{};

  if (sig.IsDoubleSig)
  {
    if (sig.Offset != 0)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 27;
      infocmnt += StrPrint(" Offset= %f", sig.Offset);
    }

    if (sig.Factor != 1)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 24;
      infocmnt += StrPrint(" Factor= %f", sig.Factor);
    }
  }
  else if (sig.IsSimpleSig == false)
  {
    // 2 type of signal
    if (sig.Offset != 0)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 27;
      infocmnt += StrPrint(" Offset= %d", (int)sig.Offset);
    }

    if (sig.Factor != 1)
    {
      infocmnt = IndentedString(offset, infocmnt);
      offset += 24;
      infocmnt += StrPrint(" Factor= %d", (int)sig.Factor);
    }
  }

  if (sig.Unit.size() > 0)
  {
    infocmnt = IndentedString(offset, infocmnt);
    infocmnt += StrPrint(" Unit:'%s'", sig.Unit.c_str());
  }

  fwriter->AppendText(infocmnt);
  fwriter->Append();
}

//(added by AdriveLivingLab)
void CiMainGenerator::WriteSimpleSigConvField(const SignalDescriptor_t &sig, bool bits, size_t padwidth)
{
  std::string dtype = "";

  if (sig.Name.find("reserved_") != std::string::npos)
  {
    return;
  }

  dtype += "  pAppMsg->" + sig.Name + " = pkt->" + sig.Name;

  size_t offset = 0;

  if (sig.IsDoubleSig)
  {

    if (sig.Factor != 1)
    {
      dtype += StrPrint(" * ") + prt_double(sig.Factor, 10);
    }

    if (sig.Offset != 0)
    {
      dtype += StrPrint(" + ") + prt_double(sig.Offset, 10);
    }
  }
  else if (sig.IsSimpleSig == false)
  {
    // 2 type of signal

    if (sig.Factor != 1)
    {
      dtype += StrPrint(" * %d", (int)sig.Factor);
    }

    if (sig.Offset != 0)
    {
      dtype += StrPrint(" + %d", (int)sig.Offset);
    }
  }

  dtype += ";";

  std::string pad = " ";

  fwriter->AppendText(dtype);
  fwriter->Append();
}

//(added by AdriveLivingLab)
void CiMainGenerator::WriteRosMsgFile(const SignalDescriptor_t &sig)
{

  if (sig.Name.find("reserved_") != std::string::npos)
  {
    return;
  }
  // if its not a trivial signal, e.g. it needs offset and factor
  if (!sig.IsSimpleSig)
  {
    // if we know its a double
    if (sig.IsDoubleSig)
    {
      fwriter->Append("float64 %s", sig.Name.c_str());
    }
    else
    {
      fwriter->Append("%s %s", PrintMsgType((int)sig.TypePhys).c_str(), sig.Name.c_str());
    }
  }
  else // simple signal
  {
    fwriter->Append("%s %s", PrintMsgType((int)sig.TypePhys).c_str(), sig.Name.c_str());
  }
}

void CiMainGenerator::WriteUnpackBody(const CiExpr_t *sgs)
{
  for (size_t num = 0; num < sgs->to_signals.size(); num++)
  {
    auto expr = sgs->to_signals[num];

    // for code shortening
    const char* sname = sgs->msg.Signals[num].Name.c_str();

    if (sgs->msg.Signals[num].Signed)
    {
      fwriter->Append("  _m->%s = %s(( %s ), %d);",
        sname, ext_sig_func_name, expr.c_str(), (int32_t)sgs->msg.Signals[num].LengthBit);
    }
    else
    {
      fwriter->Append("  _m->%s = %s;", sname, expr.c_str());
    }

    // print sigfloat conversion
    if (!sgs->msg.Signals[num].IsSimpleSig)
    {
      fwriter->Append("#ifdef %s", fdesc->gen.usesigfloat_def.c_str());

      if (sgs->msg.Signals[num].IsDoubleSig)
      {
        // for double signals (sigfloat_t) type cast
        fwriter->Append("  _m->%s = (sigfloat_t)(%s_%s_fromS(_m->%s));",
          sgs->msg.Signals[num].NameFloat.c_str(), fdesc->gen.DRVNAME.c_str(), sname, sname);
      }
      else
      {
        fwriter->Append("  _m->%s = %s_%s_fromS(_m->%s);",
          sgs->msg.Signals[num].NameFloat.c_str(), fdesc->gen.DRVNAME.c_str(), sname, sname);
      }

      fwriter->Append("#endif // %s", fdesc->gen.usesigfloat_def.c_str());
      fwriter->Append();
    }

    else if (num + 1 == sgs->to_signals.size())
    {
      // last signal without phys part, put \n manually
      fwriter->Append("");
    }
  }

  fwriter->Append("#ifdef %s", fdesc->gen.usemon_def.c_str());
  fwriter->Append("  _m->mon1.dlc_error = (dlc_ < %s_DLC);", sgs->msg.Name.c_str());
  fwriter->Append("  _m->mon1.last_cycle = GetSystemTick();");
  fwriter->Append("  _m->mon1.frame_cnt++;");
  fwriter->Append();

  if (sgs->msg.RollSig != nullptr)
  {
    // Put rolling monitor here
    fwriter->Append("#ifdef %s", fdesc->gen.useroll_def.c_str());
    fwriter->Append("  _m->mon1.roll_error = (_m->%s != _m->%s_expt);",
      sgs->msg.RollSig->Name.c_str(), sgs->msg.RollSig->Name.c_str());
    fwriter->Append("  _m->%s_expt = (_m->%s + 1) & (0x%02XU);", sgs->msg.RollSig->Name.c_str(),
      sgs->msg.RollSig->Name.c_str(), (1 << sgs->msg.RollSig->LengthBit) - 1);
    // Put rolling monitor here
    fwriter->Append("#endif // %s", fdesc->gen.useroll_def.c_str());
    fwriter->Append();
  }

  if (sgs->msg.CsmSig != nullptr)
  {
    // Put checksum check function call here
    fwriter->Append("#ifdef %s", fdesc->gen.usecsm_def.c_str());
    fwriter->Append("  _m->mon1.csm_error = (((uint8_t)GetFrameHash(_d, %s_DLC, %s_CANID, %s, %d)) != (_m->%s));",
      sgs->msg.Name.c_str(), sgs->msg.Name.c_str(), sgs->msg.CsmMethod.c_str(),
      sgs->msg.CsmOp, sgs->msg.CsmSig->Name.c_str());
    fwriter->Append("#endif // %s", fdesc->gen.usecsm_def.c_str());
    fwriter->Append();
  }

  auto Fmon_func = "FMon_" + sgs->msg.Name + "_" + fdesc->gen.drvname;

  fwriter->Append("  %s(&_m->mon1, %s_CANID);", Fmon_func.c_str(), sgs->msg.Name.c_str());

  fwriter->Append("#endif // %s", fdesc->gen.usemon_def.c_str());
  fwriter->Append();

  fwriter->Append("  return %s_CANID;", sgs->msg.Name.c_str());
}

void CiMainGenerator::WritePackStructBody(const CiExpr_t* sgs)
{
  fwriter->Append("{");
  PrintPackCommonText("cframe->Data", sgs);
  fwriter->Append("  cframe->MsgId = %s_CANID;", sgs->msg.Name.c_str());
  fwriter->Append("  cframe->DLC = %s_DLC;", sgs->msg.Name.c_str());
  fwriter->Append("  cframe->IDE = %s_IDE;", sgs->msg.Name.c_str());
  fwriter->Append("  return %s_CANID;", sgs->msg.Name.c_str());
  fwriter->Append("}");
  fwriter->Append();
}

void CiMainGenerator::WritePackArrayBody(const CiExpr_t* sgs)
{
  fwriter->Append("{");
  PrintPackCommonText("_d", sgs);
  fwriter->Append("  *_len = %s_DLC;", sgs->msg.Name.c_str());
  fwriter->Append("  *_ide = %s_IDE;", sgs->msg.Name.c_str());
  fwriter->Append("  return %s_CANID;", sgs->msg.Name.c_str());
  fwriter->Append("}");
  fwriter->Append();
}

void CiMainGenerator::PrintPackCommonText(const std::string& arrtxt, const CiExpr_t* sgs)
{
  // this function will print part of pack function
  // which is differs only by arra var name

  // pring array content clearin loop
  fwriter->Append("  uint8_t i; for (i = 0; (i < %s_DLC) && (i < 8); %s[i++] = 0);",
    sgs->msg.Name.c_str(), arrtxt.c_str());
  fwriter->Append();

  if (sgs->msg.RollSig != nullptr)
  {
    fwriter->Append("#ifdef %s", fdesc->gen.useroll_def.c_str());
    fwriter->Append("  _m->%s = (_m->%s + 1) & (0x%02XU);", sgs->msg.RollSig->Name.c_str(),
      sgs->msg.RollSig->Name.c_str(), (1 << sgs->msg.RollSig->LengthBit) - 1);
    fwriter->Append("#endif // %s", fdesc->gen.useroll_def.c_str());
    fwriter->Append();
  }

  if (sgs->msg.CsmSig != nullptr)
  {
    // code for clearing checksum
    fwriter->Append("#ifdef %s", fdesc->gen.usecsm_def.c_str());
    fwriter->Append("  _m->%s = 0U;", sgs->msg.CsmSig->Name.c_str());
    fwriter->Append("#endif // %s", fdesc->gen.usecsm_def.c_str());
    fwriter->Append();
  }

  if (sgs->msg.hasPhys)
  {
    // first step is to put code for sigfloat conversion, before
    // sigint packing to bytes.
    fwriter->Append("#ifdef %s", fdesc->gen.usesigfloat_def.c_str());

    for (size_t n = 0; n < sgs->to_signals.size(); n++)
    {
      if (sgs->msg.Signals[n].IsSimpleSig == false)
      {
        // print toS from *_phys to original named sigint (integer duplicate of signal)
        fwriter->Append("  _m->%s = %s_%s_toS(_m->%s);",
          sgs->msg.Signals[n].Name.c_str(), fdesc->gen.DRVNAME.c_str(),
          sgs->msg.Signals[n].Name.c_str(), sgs->msg.Signals[n].NameFloat.c_str());
      }
    }

    fwriter->Append("#endif // %s", fdesc->gen.usesigfloat_def.c_str());
    fwriter->Append();
  }

  for (size_t i = 0; i < sgs->to_bytes.size(); i++)
  {
    if (sgs->to_bytes[i].size() < 2)
    {
      continue;
    }

    fwriter->Append("  %s[%d] |= %s;", arrtxt.c_str(), i, sgs->to_bytes[i].c_str());
  }

  fwriter->Append("");

  if (sgs->msg.CsmSig != nullptr)
  {
    // code for getting checksum value and putting it in array
    fwriter->Append("#ifdef %s", fdesc->gen.usecsm_def.c_str());

    fwriter->Append("  _m->%s = ((uint8_t)GetFrameHash(%s, %s_DLC, %s_CANID, %s, %d));",
      sgs->msg.CsmSig->Name.c_str(), arrtxt.c_str(), sgs->msg.Name.c_str(),
      sgs->msg.Name.c_str(), sgs->msg.CsmMethod.c_str(), sgs->msg.CsmOp);

    fwriter->Append("  %s[%d] |= %s;", arrtxt.c_str(), sgs->msg.CsmByteNum, sgs->msg.CsmToByteExpr.c_str());

    fwriter->Append("#endif // %s", fdesc->gen.usecsm_def.c_str());
    fwriter->Append();
  }
}
