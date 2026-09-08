How to recover missing data from base station offline buffer

"Purpose
Recover wind records buffered on the Base Station during outages and manually insert them into the canonical Pearl Google Sheet.

A) Connect to Base Station
• Go to the Base Station
• Connect USB cable to laptop
• Power on the Base Station (UPS or mains)

B) Open Serial Monitor
• From project directory:
  pio device monitor
• This is the command interface
• Type HELP to list available commands (case-insensitive)

C) Dump Buffered Records
• Dump a cnt range:
  dump cnt <start> <end>
• Example:
  dump cnt 20 51
• Dump automatically spans buffer A/B if needed

D) Paste into “Test missing data” sheet
• Paste CSV output directly (columns already match Pearl)
• Do NOT reorder columns
• ts is Unix time → convert before inserting into Pearl

E) Convert ts → Date/Time
• Use helper column (Date/Time)
• Formula:
  =A2/86400 + DATE(1970,1,1)
• Copy → Paste values only over ts column

F) Final Insert
• Check the sort by cnt descending (highest cnt = most recent, canonical)
• Cut/paste rows into Pearl sheet at the correct gap
"																									
																									
																									