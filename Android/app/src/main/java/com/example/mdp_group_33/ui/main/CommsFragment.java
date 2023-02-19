package com.example.mdp_group_33.ui.main;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProviders;


import com.example.mdp_group_33.R;
import com.google.android.material.floatingactionbutton.FloatingActionButton;

import java.nio.charset.Charset;
import java.util.ArrayList;

public class CommsFragment extends Fragment {

    private static final String ARG_SECTION_NUMBER = "section_number";

    private PageViewModel pageViewModel;

    SharedPreferences messageReceived, messageSent;

    FloatingActionButton sendBtn;
    private static TextView messageSentTextView;
    private static TextView messageReceivedTextView;
    private EditText typeBoxEditText;

    public static CommsFragment newInstance(int index) {
        CommsFragment fragment = new CommsFragment();
        Bundle bundle = new Bundle();
        bundle.putInt(ARG_SECTION_NUMBER, index);
        fragment.setArguments(bundle);
        return fragment;
    }


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        pageViewModel = ViewModelProviders.of(this).get(PageViewModel.class);
        int index = 1;
        if (getArguments() != null) {
            index = getArguments().getInt(ARG_SECTION_NUMBER);
        }
        pageViewModel.setIndex(index);
    }

    @Override
    public View onCreateView(
            @NonNull LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.activity_comms, container, false);

        sendBtn = root.findViewById(R.id.messageButton);

        messageSentTextView = root.findViewById(R.id.messageSentTextView);
        messageSentTextView.setMovementMethod(new ScrollingMovementMethod());

        messageReceivedTextView = root.findViewById(R.id.messageReceivedTextView);
        messageReceivedTextView.setMovementMethod(new ScrollingMovementMethod());

        typeBoxEditText = root.findViewById(R.id.typeBoxEditText);

        messageSent = getActivity().getSharedPreferences("Message Sent", Context.MODE_PRIVATE);
        messageReceived = getActivity().getSharedPreferences("Message Received", Context.MODE_PRIVATE);

        sendBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                String sentText = "" + typeBoxEditText.getText().toString();

                SharedPreferences.Editor editor = messageSent.edit();
                editor.putString("message", messageSent.getString("message", "") + '\n' + sentText);
                editor.commit();
                messageSentTextView.setText(messageSent.getString("message", ""));
                typeBoxEditText.setText("");

                if (BTCon.BluetoothConnectionStatus == true) {
                    byte[] bytes = sentText.getBytes(Charset.defaultCharset());
                    BTCon.write(bytes);
                }
            }
        });

        return root;
    }

    public static TextView getMessageReceived() { return messageReceivedTextView; }

}