Module Program
    <STAThreadAttribute()>
    Public Sub Main()
        Dim mainForm As MainForm = (New MainForm)
        mainForm.MainForm()
        System.Windows.Forms.Application.EnableVisualStyles()
        System.Windows.Forms.Application.Run(mainForm)
    End Sub
End Module
